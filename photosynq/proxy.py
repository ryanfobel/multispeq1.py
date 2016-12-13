import uuid

from path_helpers import path
import numpy as np
try:
    from base_node_rpc.proxy import ConfigMixinBase, StateMixinBase
    import arduino_helpers.hardware.teensy as teensy

    from .node import (Proxy as _Proxy, I2cProxy as _I2cProxy,
                       SerialProxy as _SerialProxy)
    from .config import Config
    from .state import State


    HW_TCDS_ADDR = 0x40009000
    TCD_RECORD_DTYPE = [('SADDR', 'uint32'),
                        ('SOFF', 'uint16'),
                        ('ATTR', 'uint16'),
                        ('NBYTES', 'uint32'),
                        ('SLAST', 'uint32'),
                        ('DADDR', 'uint32'),
                        ('DOFF', 'uint16'),
                        ('CITER', 'uint16'),
                        ('DLASTSGA', 'uint32'),
                        ('CSR', 'uint16'),
                        ('BITER', 'uint16')]


    class ConfigMixin(ConfigMixinBase):
        @property
        def config_class(self):
            return Config

        @property
        def uuid(self):
            '''
            Returns
            -------

                (uuid.UUID) : UUID constructed from the [Unique Identification
                    Register][1] (12.2.19 page 265).


            [1]: https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
            '''
            return uuid.UUID(bytes=np.array(self._uuid(),
                                            dtype='uint8').tostring())


    class StateMixin(StateMixinBase):
        @property
        def state_class(self):
            return State


    class ProxyMixin(ConfigMixin, StateMixin):
        '''
        Mixin class to add convenience wrappers around methods of the generated
        `node.Proxy` class.

        For example, expose config and state getters/setters as attributes.
        '''
        host_package_name = str(path(__file__).parent.name.replace('_', '-'))

        def __init__(self, *args, **kwargs):
            super(ProxyMixin, self).__init__(*args, **kwargs)
            self.init_dma()

        def init_dma(self):
            '''
            Initialize eDMA engine.  This includes:

             - Enabling clock gating for DMA and DMA mux.
             - Resetting all DMA channel transfer control descriptors.

            See the following sections in [K20P64M72SF1RM][1] for more
            information:

             - (12.2.13/256) System Clock Gating Control Register 6 (SIM_SCGC6)
             - (12.2.14/259) System Clock Gating Control Register 7 (SIM_SCGC7)
             - (21.3.17/415) TCD registers

            [1]: https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
            '''
            from .SIM import R_SCGC6, R_SCGC7

            # Enable DMA-related clocks in clock-gating configuration
            # registers.
            # SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
            self.update_sim_SCGC6(R_SCGC6(DMAMUX=True))
            # SIM_SCGC7 |= SIM_SCGC7_DMA;
            self.update_sim_SCGC7(R_SCGC7(DMA=True))

            # Reset all DMA transfer control descriptor registers (i.e., set to
            # 0).
            for i in xrange(self.dma_channel_count()):
                self.reset_dma_TCD(i)

        def DMA_TCD(self, dma_channel):
            '''
            Parameters
            ----------
            dma_channel : int
                DMA channel for which to read the Transfer Control Descriptor.

            Returns
            -------
            pandas.DataFrame
                Table of Transfer Control Descriptor fields with corresponding
                values.

                In addition, several reference columns are included.  For
                example, the ``page`` column indicates the page in the
                [reference manual][1] where the respective register field is
                described.

            [1]: http://www.freescale.com/products/arm-processors/kinetis-cortex-m/adc-calculator:ADC_CALCULATOR
            '''
            from arduino_rpc.protobuf import resolve_field_values
            import arduino_helpers.hardware.teensy.dma as dma
            from .DMA import TCD

            tcd = TCD.FromString(self.read_dma_TCD(dma_channel).tostring())
            df_tcd = resolve_field_values(tcd)
            return (df_tcd[['full_name', 'value']].dropna()
                    .join(dma.TCD_DESCRIPTIONS, on='full_name')
                    [['full_name', 'value', 'short_description', 'page']]
                    .sort_values(['page','full_name']))

        def DMA_registers(self):
            '''
            Returns
            -------
            pandas.DataFrame
                Table of DMA configuration fields with corresponding values.

                In addition, several reference columns are included.  For
                example, the ``page`` column indicates the page in the
                [reference manual][1] where the respective register field is
                described.

            [1]: http://www.freescale.com/products/arm-processors/kinetis-cortex-m/adc-calculator:ADC_CALCULATOR
            '''
            from arduino_rpc.protobuf import resolve_field_values
            import arduino_helpers.hardware.teensy.dma as dma
            from .DMA import Registers

            dma_registers = (Registers
                             .FromString(self.read_dma_registers().tostring()))
            df_dma = resolve_field_values(dma_registers)
            return (df_dma.dropna(subset=['value'])
                    .join(dma.REGISTERS_DESCRIPTIONS, on='full_name')
                    [['full_name', 'value', 'short_description', 'page']])

        def tcd_msg_to_struct(self, tcd_msg):
            '''
            Convert Transfer Control Descriptor from Protocol Buffer message
            encoding to raw structure, i.e., 32 bytes starting from `SADDR`
            field.

            See 21.3.17/415 in the [manual][1] for more details.

            Parameters
            ----------
            tcd_msg : photosynq.DMA.TCD
                Transfer Control Descriptor Protocol Buffer message.

            Returns
            -------
            numpy.ndarray(dtype=TCD_RECORD_DTYPE)
                Raw Transfer Control Descriptor structure (i.e., 32 bytes
                starting from ``SADDR`` field) as a :class:`numpy.ndarray` of
                type :data:`TCD_RECORD_DTYPE`.

            See also
            --------
            :meth:`tcd_struct_to_msg`

            [1]: https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
            '''
            # Copy TCD to device so that we can extract the raw bytes from
            # device memory (raw TCD bytes are read into `tcd0`.
            #
            # **TODO**:
            #  - Modify `Photosynq/DMA.h::serialize_TCD` and
            #  `Photosynq/DMA.h::update_TCD` functions to work offline.
            #      * Operate on variable by reference, on-device use actual register.
            #  - Add `arduino_helpers.hardware.teensy` function to convert
            #    between TCD protobuf message and binary TCD struct.
            self.update_dma_TCD(0, tcd_msg)
            return (self.mem_cpy_device_to_host(HW_TCDS_ADDR, 32)
                    .view(TCD_RECORD_DTYPE)[0])

        def tcd_struct_to_msg(self, tcd_struct):
            '''
            Convert Transfer Control Descriptor from raw structure (i.e., 32
            bytes starting from ``SADDR`` field) to Protocol Buffer message
            encoding.

            See 21.3.17/415 in the [manual][1] for more details.

            Parameters
            ----------
            tcd_struct : numpy.ndarray
                Raw Transfer Control Descriptor structure (i.e., 32 bytes
                starting from ``SADDR`` field)

            Returns
            -------
            photosynq.DMA.TCD
                Transfer Control Descriptor Protocol Buffer message.

            See also
            --------
            :meth:`tcd_msg_to_struct`

            [1]: https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
            '''
            from .DMA import TCD

            # Copy TCD structure to device so that we can extract the
            # serialized protocol buffer representation from the device.
            #
            # **TODO**:
            #  - Modify `Photosynq/DMA.h::serialize_TCD` and
            #    `Photosynq/DMA.h::update_TCD` functions to operate on
            #    arbitrary source address (instead of hard-coded address of TCD
            #    register).
            #      * Operate on variable by reference, on-device use actual register.
            #  - Add `arduino_helpers.hardware.teensy` function to convert between TCD
            #    protobuf message and binary TCD struct.
            self.mem_cpy_host_to_device(HW_TCDS_ADDR, tcd_struct.tostring())
            return TCD.FromString(self.read_dma_TCD(0).tostring())

        def analog_reads(self, adc_channel, sample_count, resolution=None,
                         average_count=1, sampling_rate_hz=None,
                         differential=False, gain_power=0,
                         adc_num=teensy.ADC_0):
            '''
            Read multiple samples from a single ADC channel, using the minimum
            conversion rate for specified sampling parameters.

            The reasoning behind selecting the minimum conversion rate is that
            we expect it to lead to the lowest noise possible while still
            matching the specified requirements.
            **TODO** Should this be handled differently?

            This function uses the following mutator methods:

             - :meth:`setReference` (C++)
                 * Set the reference voltage based on whether or not
                   differential is selected.

                   On the Teensy 3.2 architecture, the 1.2V reference voltage
                   *must* be used when operating in differential (as opposed to
                   singled-ended) mode.

             - :meth:`update_adc_registers` (C++)
                 * Apply ADC ``CFG*`` register settings.

                   ADC ``CFG*`` register settings are determined by:

                     - :data:`average_count`
                     - :data:`differential`
                     - :data:`gain_power` (PGA only)
                     - :data:`resolution`
                     - :data:`sampling_rate_hz`

             - :meth:`setAveraging` (C++)
                 * Apply ADC hardware averaging setting using Teensy ADC
                   library API.

                   We use the Teensy API here because it handles calibration,
                   etc. automatically.

             - :meth:`setResolution` (C++)
                 * Apply ADC resolution setting using Teensy ADC library API.

                   We use the Teensy API here because it handles calibration,
                   etc. automatically.

            Parameters
            ----------
            adc_channel : string
                ADC channel to measure (e.g., ``'A0'``, ``'PGA0'``, etc.).
            sample_count : int
                Number of samples to measure.
            resolution : int,optional
                Bit resolution to sample at.  Must be one of: 8, 10, 12, 16.
            average_count : int,optional
                Hardware average count.
            sampling_rate_hz : int,optional
                Sampling rate.  If not specified, sampling rate will be based
                on minimum conversion rate based on remaining ADC settings.
            differential : bool,optional
                If ``True``, use differential mode.  Otherwise, use single-ended
                mode.
                .. note::
                    **Differential mode** is automatically enabled for ``PGA*``
                    measurements (e.g., :data:`adc_channel=='PGA0'`).
            gain_power : int,optional
                When measuring a ``'PGA*'`` channel (also implies differential
                mode), apply a gain of ``2^gain_power`` using the hardware
                programmable amplifier gain.
            adc_num : int,optional
                The ADC to use for the measurement (default is
                ``teensy.ADC_0``).

            Returns
            -------
            sampling_rate_hz : int
                Number of samples per second.
            adc_settings : pandas.Series
                ADC settings used.
            df_volts : pandas.DataFrame
                Voltage readings (based on reference voltage and gain).
            df_adc_results : pandas.DataFrame
                Raw ADC values (range depends on resolution, i.e.,
                ``adc_settings['Bit-width']``).
            '''
            from .adc_sampler import AdcSampler, DEFAULT_ADC_CONFIGS
            import photosynq.ADC as ADC

            # Select ADC settings to achieve minimum conversion rate for
            # specified resolution, mode (i.e., single-ended or differential),
            # and number of samples to average per conversion (i.e., average
            # count).
            bit_width = resolution

            if 'PGA' in adc_channel:
                differential = True

            if differential:
                if (resolution is not None) and ((resolution < 16) and not
                                                 (resolution & 0x01)):
                    # An even number of bits was specified for resolution in
                    # differential mode. However, bit-widths are actually
                    # increased by one bit, where the additional bit indicates
                    # the sign of the result.
                    bit_width += 1

            elif gain_power > 0:
                raise ValueError('Programmable gain amplification is only '
                                 'valid in differential mode.')
            mode = 'differential' if differential else 'single-ended'

            # Build up a query mask based on specified options.
            query = ((DEFAULT_ADC_CONFIGS.AverageNum == average_count)
                    & (DEFAULT_ADC_CONFIGS.Mode == mode))
            if resolution is not None:
                query &= (DEFAULT_ADC_CONFIGS['Bit-width'] == bit_width)
            if sampling_rate_hz is not None:
                query &= (DEFAULT_ADC_CONFIGS.conversion_rate >=
                          sampling_rate_hz)

            # Use prepared query mask to select matching settings from the
            # table of valid ADC configurations.
            matching_settings = DEFAULT_ADC_CONFIGS.loc[query]

            # Find and select the ADC configuration with the minimum conversion
            # rate.
            # **TODO** The reasoning behind selecting the minimum conversion
            # rate is that we expect it to lead to the lowest noise possible
            # while still matching the specified requirements.
            min_match_index = matching_settings['conversion_rate'].argmin()
            adc_settings = matching_settings.loc[min_match_index].copy()

            if resolution is None:
                resolution = int(adc_settings['Bit-width'])

            # Set the reference voltage based on whether or not differential is
            # selected.
            if differential:
                # On the Teensy 3.2 architecture, the 1.2V reference voltage
                # *must* be used when operating in differential (as opposed to
                # singled-ended) mode.
                self.setReference(teensy.ADC_REF_1V2, adc_num)
                reference_V = 1.2
            else:
                self.setReference(teensy.ADC_REF_3V3, adc_num)
                reference_V = 3.3
            # Verify that a valid gain value has been specified.
            assert(gain_power >= 0 and gain_power < 8)
            adc_settings['gain_power'] = int(gain_power)

            # Construct a Protocol Buffer message according to the selected ADC
            # configuration settings.
            adc_registers = \
                ADC.Registers(CFG2=
                              ADC.R_CFG2(MUXSEL=ADC.R_CFG2.B,
                                         ADACKEN=
                                         int(adc_settings['CFG2[ADACKEN]']),
                                         ADLSTS=
                                         int(adc_settings['CFG2[ADLSTS]']),
                                         ADHSC=
                                         int(adc_settings['CFG2[ADHSC]'])),
                              CFG1=
                              ADC.R_CFG1(ADLSMP=
                                         int(adc_settings['CFG1[ADLSMP]']),
                                         ADICLK=
                                         int(adc_settings['CFG1[ADICLK]']),
                                         ADIV=
                                         int(adc_settings['CFG1[ADIV]'])))
            if 'PGA' in adc_channel:
                adc_registers.PGA.PGAG = adc_settings.gain_power
                adc_registers.PGA.PGAEN = True

            # Apply ADC `CFG*` register settings.
            self.update_adc_registers(adc_num, adc_registers)

            # Apply non-`CFG*` ADC settings using Teensy ADC library API.
            # We use the Teensy API here because it handles calibration, etc.
            # automatically.
            self.setAveraging(average_count, adc_num)
            self.setResolution(resolution, adc_num)

            if sampling_rate_hz is None:
                # By default, use a sampling rate that is 90% of the maximum
                # conversion rate for the selected ADC settings.
                #
                # **TODO** We arrived at this after a handful of empirical
                # tests. We think this is necessary due to the slight deviation
                # of the computed conversion rates (see TODO in
                # `adc_sampler.get_adc_configs`) from those computed by the
                # [Freescale ADC calculator][1].
                #
                # [1]: http://www.freescale.com/products/arm-processors/kinetis-cortex-m/adc-calculator:ADC_CALCULATOR
                sampling_rate_hz = (int(.9 * adc_settings.conversion_rate) &
                                    0xFFFFFFFE)

            # Create `AdcSampler` for:
            #  - The specified sample count.
            #  - The specified channel.
            #      * **N.B.,** The `AdcSampler` supports scanning multiple
            #        channels, but in this function, we're only reading from a
            #        single channel.
            channels = [adc_channel]

            # **Creation of `AdcSampler` object initializes DMA-related
            # registers**.
            adc_sampler = AdcSampler(self, channels, sample_count)
            adc_sampler.reset()
            adc_sampler.start_read(sampling_rate_hz)
            dtype = 'int16' if differential else 'uint16'
            # **TODO** We should probably *explicitly* wait until read is done.
            # (or stream?).
            df_adc_results = adc_sampler.get_results().astype(dtype)
            df_volts = reference_V * (df_adc_results /
                                    (1 << (resolution +
                                           adc_settings.gain_power)))
            return sampling_rate_hz, adc_settings, df_volts, df_adc_results


    class Proxy(ProxyMixin, _Proxy):
        pass

    class I2cProxy(ProxyMixin, _I2cProxy):
        pass

    class SerialProxy(ProxyMixin, _SerialProxy):
        pass

except (ImportError, TypeError):
    Proxy = None
    I2cProxy = None
    SerialProxy = None
