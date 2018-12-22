from .logging_logger import logger


try:
    import frozendict  # @UnusedImport @UnresolvedImport
except:
    msg = 'frozendict not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     sudo apt install python-frozendict'
    raise Exception(msg)


try:
    from ruamel import yaml # @UnusedImport
except:
    msg = 'ruamel.yaml not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     sudo apt install python-ruamel.yaml'
    raise Exception(msg)


from .constants import * 

from .augmented_reality_utils import *
from .bag_info import *
from .bag_logs import *
from .bag_reading import *
from .bag_visualization import *
from .bag_writing import *
from .caching import *
# from .cli import *
from .col_logging import *
from .constants import *
from .contracts_ import *
from .dates import *
from .detect_environment import *
from .disk_hierarchy import *
from .download import *
from .exceptions import *
from .exception_utils import *
from .expand_variables import *
from .file_utils import *
from .friendly_path_imp import *
from .fuzzy import *
from .image_composition import *
from .image_conversions import *
from .image_jpg_create import *
from .image_rescaling import *
from .image_timestamps import *
from .image_writing import *
from .image_operations import *

from .instantiate_utils import *
from .ipython_utils import *
from .jpg import *
from .locate_files_impl import *
from .logging_logger import *
from .memoization import *
from .mkdirs import *
from .networking import *
from .parameters import *
from .path_utils import *
from .read_package_xml import *
from .safe_pickling import *
from .system_cmd_imp import *
from .test_hash import *
from .text_utils import *
from .timeit import *
from .type_checks import *
from .wildcards import *
from .wrap_main import *
from .yaml_pretty import *
from .yaml_wrap import *

# Make sure that all variables look like they are 
# in the duckietown_utils module, not duckietown_utils
__all__ = []
for c in list(locals()):
    v = eval(c)
    if hasattr(v, '__module__'):
        if v.__module__.startswith('duckietown_utils'):
            v.__module__ = 'duckietown_utils'
            __all__.append(c)
            


