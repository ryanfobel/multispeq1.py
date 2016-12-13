from nose.tools import with_setup
import numpy as np
import multispeq1 as tr


def setup_func():
    global proxy
    proxy = tr.SerialProxy()


def teardown_func():
    global proxy
    del proxy


@with_setup(setup_func, teardown_func)
def test_rms():
    for N_i in (16, 32, 1024, 8 * 1024):
        for auto_mean_ij in (True, False):
            yield check_rms, N_i, auto_mean_ij


def check_rms(N, auto_mean):
    '''
    Compare teensy root mean square vs numpy calculation.
    '''
    data = np.arange(N)
    data_mean = data.mean()
    py_rms = np.sqrt(np.sum(((data - data_mean) ** 2)) / N)

    if auto_mean:
        rms_func = proxy.compute_uint16_rms_auto_mean
    else:
        rms_func = lambda *args: proxy.compute_uint16_rms(*(args +
                                                            (data_mean, )))

    # Select CHUNK_SIZE as a multiple of 2, allow up to 16 bytes for packet
    # header.
    CHUNK_SIZE = 2 * (proxy.max_serial_payload_size() // 2) - 16

    data_addr = proxy.mem_alloc(2 * N)
    try:
        proxy.mem_fill_uint16(data_addr, 0, N)

        for i in xrange(0, int(np.ceil(2 * N / float(CHUNK_SIZE)))):
            proxy.mem_cpy_host_to_device(data_addr + (CHUNK_SIZE * i),
                                         np.arange(CHUNK_SIZE * i / 2,
                                                   min(CHUNK_SIZE * (i + 1) /
                                                       2, N),
                                                   dtype='uint16')
                                         .view('uint8'))

        teensy_rms = rms_func(data_addr, N)
        assert(np.isclose(py_rms, teensy_rms))
    finally:
        proxy.mem_free(data_addr)
