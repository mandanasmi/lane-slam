import os

from . import logger
from .exceptions import DTConfigException
from .locate_files_impl import locate_files
from .path_utils import expand_all


class DuckietownConstants():
    DUCKIETOWN_ROOT_variable = 'DUCKIETOWN_ROOT'
    DUCKIEFLEET_ROOT_variable = 'DUCKIEFLEET_ROOT'
    
    # If the environment variable is not set, use these: 
    duckiefleet_root_defaults = [
        '~/duckiefleet',
        '~/duckiefleet-fall2017',
    ]
    
    DUCKIETOWN_DATA_variable = 'DUCKIETOWN_DATA'
    DUCKIETOWN_CONFIG_SEQUENCE_variable = 'DUCKIETOWN_CONFIG_SEQUENCE'
    
    # inside DUCKIEFLEET_ROOT
#     scuderia_filename = 'scuderia.yaml'
    machines_path_rel_to_root = 'catkin_ws/src/00-infrastructure/duckietown/machines'
    
    use_cache_for_algos =  False
    
    enforce_no_tabs = True
    enforce_naming_conventions = True
    
    # The rules for good readmes do not apply to these packages
    good_readme_exceptions = ['apriltags_ros', 'apriltags', 'duckietown', 'isam']

    
def get_duckietown_root():
    """ Returns the path of DUCKIETOWN_ROOT and checks it exists """
    return _get_dir(DuckietownConstants.DUCKIETOWN_ROOT_variable)


def get_duckiefleet_root():
    """ 
        Returns the path of DUCKIEFLEET_ROOT and checks it exists.
        Raises DTConfigException. 
    """

    # If the environment variable is set:    
    vname = DuckietownConstants.DUCKIEFLEET_ROOT_variable
    if vname in os.environ:
        return _get_dir(vname)
    else:
        msg = 'The environment variable %s is not defined,' % vname
        msg += ' so I will look for the default directories.'  
        logger.info(msg)
        
        defaults =  DuckietownConstants.duckiefleet_root_defaults
        found = []
        for d in defaults:
            d2 = expand_all(d)
            if os.path.exists(d2):
                found.append(d2)
        if not found:
            msg = 'Could not find any of the default directories:'
            for d in defaults:
                msg += '\n- %s' % d
            raise DTConfigException(msg)
        
        if len(found) > 1:
            msg = 'I found more than one match for the default directories:'
            for d in found:
                msg += '\n- %s' % d
            raise DTConfigException(msg)
        
        return found[0]        

def get_duckietown_data():
    """ Returns the path of DUCKIETOWN_DATA and checks it exists """
    return _get_dir(DuckietownConstants.DUCKIETOWN_DATA_variable)

# def get_duckietown_tmp_video_process():
#     """ Returns a suitable dir to do temporary video processing. """
#     d = get_duckietown_root()
#     d =  os.path.join(d, 'caches', 'tmp')
#     d8n_mkdirs_thread_safe(d)
#     return d
    

def get_duckietown_local_log_downloads():
    """ Returns the directory to use for local downloads of logs"""
    d = '/tmp/duckietown-download'
    if not os.path.exists(d):
        os.makedirs(d)
    return d
    

def get_machines_files_path():
    ''' Gets the path to the machines file. It might not exist. '''
    duckietown_root = get_duckietown_root()
    machines = os.path.join(duckietown_root, DuckietownConstants.machines_path_rel_to_root)
    return machines

def get_catkin_ws_src():
    ''' Returns the path to the src/ dir in catkin_ws '''
    duckietown_root = get_duckietown_root()
    machines = os.path.join(duckietown_root, 'catkin_ws/src')
    return machines

def get_list_of_packages_in_catkin_ws():
    """
        Returns an ordered dictionary <package name>: <package dir>
        of packages that exist in catkin_ws/src.
    
        Raises DTConfigException if $DUCKIETOWN_ROOT is not set.
    """
    src = get_catkin_ws_src()
    package_files = locate_files(src, 'package.xml')
    results = {}
    for p in package_files:
        dn = os.path.dirname(p)
        entry = os.path.basename(dn)
        if not is_ignored_by_catkin(dn):
            results[entry] = dn
        else:
            #logger.debug('Not considering %s' % dn)
            pass
    # We expect at least these two packages
    if not 'duckietown' in results:
        raise ValueError('Could not find the duckietown ROS package.')
    if not 'what_the_duck' in results:
        raise ValueError('Could not find what_the_duck') 
    return results

def is_ignored_by_catkin(dn):
    """ Returns true if the directory is inside one with CATKIN_IGNORE """
    while dn != '/':
        i = os.path.join(dn, "CATKIN_IGNORE")
        if os.path.exists(i):
            return True
        dn = os.path.dirname(dn)
    return False

def get_scuderia_path():
    ''' Gets the path to the scuderia files. It might not exist. '''
    duckiefleet = get_duckiefleet_root()
    machines = os.path.join(duckiefleet, DuckietownConstants.scuderia_filename)
    return machines



def _get_dir(variable_name):
    """ 
        Raises DTConfigException if it does not exist or the environment variable is not set.
    """
    if not variable_name in os.environ:
        msg = 'Environment variable %r not defined.' % variable_name
        raise DTConfigException(msg)
    
    fn = expand_all(os.environ[variable_name])
    
    if not os.path.exists(fn):
        msg = 'Could not get %s: dir does not exist: %s' % (variable_name, fn)
        raise DTConfigException(msg)
    
    return fn
    