import uuid

from path_helpers import path
import numpy as np
try:
    from base_node_rpc.proxy import ConfigMixinBase, StateMixinBase
    import arduino_helpers.hardware.teensy as teensy

    from .node import (Proxy as _Proxy, I2cProxy as _I2cProxy,
                       SerialProxy as _SerialProxy)


    class ProxyMixin(object):
        '''
        Mixin class to add convenience wrappers around methods of the generated
        `node.Proxy` class.

        For example, expose config and state getters/setters as attributes.
        '''
        host_package_name = str(path(__file__).parent.name.replace('_', '-'))

        def __init__(self, *args, **kwargs):
            super(ProxyMixin, self).__init__(*args, **kwargs)

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
