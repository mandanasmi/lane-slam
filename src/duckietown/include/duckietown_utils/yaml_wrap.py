from collections import OrderedDict
import fnmatch
import os

from duckietown_utils import logger

from .constants import get_catkin_ws_src, get_duckiefleet_root, \
    get_duckietown_data, get_duckietown_local_log_downloads
from .contracts_ import contract
from .exception_utils import check_isinstance
from .exception_utils import raise_wrapped
from .exceptions import DTConfigException
from .friendly_path_imp import friendly_path
from .instantiate_utils import indent
from .locate_files_impl import locate_files
from .yaml_pretty import yaml_load
from duckietown_utils.yaml_pretty import yaml_load_plain


def yaml_write_to_file(ob, filename):
    from duckietown_utils.yaml_pretty import yaml_dump_pretty
    from duckietown_utils.file_utils import write_data_to_file
    try:
        s = yaml_dump_pretty(ob)
    except:
        # todo : add log
        import yaml as alt
        s = alt.dump(ob)
    write_data_to_file(s, filename)
    
    
def yaml_load_file(filename):
    if not os.path.exists(filename):
        msg = 'File does not exist: %s' % friendly_path(filename)
        raise ValueError(msg)
    with open(filename) as f:
        contents = f.read()
    return interpret_yaml_file(filename, contents, lambda _filename, data: data)

def interpret_yaml_file(filename, contents, f, plain_yaml=False):
    """ 
        f is a function that takes
        
            f(filename, data)
            
        f can raise KeyError, or DTConfigException """
    try:
        from ruamel.yaml.error import YAMLError
        
        try:
            if plain_yaml:
                data = yaml_load_plain(contents)
            else:
                data = yaml_load(contents)
        except YAMLError as e:
            msg = 'Invalid YAML content:'
            raise_wrapped(DTConfigException, e, msg, compact=True)
        except TypeError as e:
            msg = 'Invalid YAML content; this usually happens '
            msg += 'when you change the definition of a class.'
            raise_wrapped(DTConfigException, e, msg, compact=True)
        try:  
            return f(filename, data)
        except KeyError as e:
            msg = 'Missing field "%s".' % e.args[0]
            raise DTConfigException(msg)
     
    except DTConfigException as e:
        msg = 'Could not interpret the contents of the file using %s()\n' % f.__name__
        msg += '   %s\n' % friendly_path(filename)
        msg += 'Contents:\n' + indent(contents[:300], ' > ')
        raise_wrapped(DTConfigException, e, msg, compact=True) 

def get_config_sources():
    sources = []
    # We look in $DUCKIETOWN_ROOT/catkin_ws/src
    sources.append(get_catkin_ws_src())
    # then we look in $DUCKIETOWN_FLEET
    sources.append(get_duckiefleet_root())
    
    return sources
 
@contract(pattern=str, sources='seq(str)')
def look_everywhere_for_config_files(pattern, sources):
    """
        Looks for all the configuration files by the given pattern.    
        Returns a dictionary filename -> contents.
    """
    check_isinstance(sources, list)
    
    logger.debug('Reading configuration files with pattern %s.' % pattern)
 
    results = OrderedDict()
    for s in sources:
        filenames = locate_files(s, pattern)
        for filename in filenames:
            contents = open(filename).read()
            results[filename] = contents
        logger.debug('%4d files found in %s' % (len(results), friendly_path(s)))
    return results

@contract(pattern=str, all_yaml='dict(str:str)')
def look_everywhere_for_config_files2(pattern, all_yaml):
    """
        Looks for all the configuration files by the given pattern.    
        Returns a dictionary filename -> contents.
        
        all_yaml = filename -> contents.
    """

    results = OrderedDict()
    for filename, contents in all_yaml.items():
        if fnmatch.fnmatch(filename, pattern):
            results[filename] = contents

    logger.debug('%4d configuration files with pattern %s.' 
                 % (len(results), pattern))
    return results

def look_everywhere_for_bag_files(pattern='*.bag'):
    """
        Looks for all the bag files    
        Returns a list of basename -> filename.
    """
    sources = []
    # We look in $DUCKIETOWN_ROOT/catkin_ws/src
    # sources.append(get_catkin_ws_src())
    # then we look in $DUCKIETOWN_FLEET
    sources.append(get_duckiefleet_root())
    sources.append(get_duckietown_data())
    # downloads 
    p = get_duckietown_local_log_downloads()
    if os.path.exists(p):
        sources.append(p)
    
    logger.debug('Looking for files with pattern %s...' % pattern)
    
    results = OrderedDict()
    for s in sources:
        filenames = locate_files(s, pattern)
        logger.debug('%5d files in %s' % (len(filenames), friendly_path(s)))
        for filename in filenames:
            basename, _ = os.path.splitext(os.path.basename(filename))
            if basename in results:
                one = filename
                two = results[basename]
                if not same_file_content(one, two):
                    msg = 'Two bags with same name but different content:\n%s\n%s' %(one, two)
                    raise DTConfigException(msg)
                else:
                    msg = 'Two copies of bag found:\n%s\n%s' %(one, two)
                    logger.warn(msg)
                    continue
            results[basename] = filename
    return results

def same_file_content(a, b):
    """ Just check the size """
    s1 = os.stat(a).st_size
    s2 = os.stat(b).st_size
    return s1==s2

    
