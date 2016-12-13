from collections import OrderedDict
import sys
from importlib import import_module

from paver.easy import task, needs, path, sh, cmdopts, options
from paver.setuputils import setup, find_package_data, install_distutils_tasks
try:
    from base_node_rpc.pavement_base import *
except ImportError:
    import warnings

    warnings.warn('Could not import `base_node_rpc` (expected during '
                  'install).')

sys.path.insert(0, '.')
import version
install_distutils_tasks()

DEFAULT_ARDUINO_BOARDS = []  #['mega2560']
PROJECT_PREFIX = [d for d in path('.').dirs()
                  if d.joinpath('Arduino').isdir()
                  and d.name not in ('build', )][0].name
module_name = PROJECT_PREFIX
package_name = module_name.replace('_', '-')
rpc_module = import_module(PROJECT_PREFIX)
VERSION = version.getVersion()
URL='http://github.com/wheeler-microfluidics/%s.git' % package_name
PROPERTIES = OrderedDict([('package_name', package_name),
                          ('display_name', package_name),
                          ('manufacturer', 'Wheeler Lab'),
                          ('software_version', VERSION),
                          ('url', URL)])
LIB_PROPERTIES = PROPERTIES.copy()
LIB_PROPERTIES.update(OrderedDict([('author', 'Christian Fobel'),
                                   ('author_email', 'christian@fobel.net'),
                                   ('short_description', 'Template project '
                                    'demonstrating use of Arduino base node '
                                    'RPC framework.'),
                                   ('version', VERSION),
                                   ('long_description', ''),
                                   ('category', 'Communication'),
                                   ('architectures', 'avr')]))

options(
    pointer_width=32,
    rpc_module=rpc_module,
    PROPERTIES=PROPERTIES,
    LIB_PROPERTIES=LIB_PROPERTIES,
    base_classes=['BaseNodeSerialHandler',],
    rpc_classes=['multispeq1::Node'],
    DEFAULT_ARDUINO_BOARDS=DEFAULT_ARDUINO_BOARDS,
    setup=dict(name=package_name,
               version=VERSION,
               description=LIB_PROPERTIES['short_description'],
               author='Christian Fobel',
               author_email='christian@fobel.net',
               url=URL,
               license='GPLv2',
               install_requires=['base-node-rpc>=0.12.post11'],
               include_package_data=True,
               packages=[str(PROJECT_PREFIX)]))
