from duckietown_utils import logger

from .friendly_path_imp import friendly_path
from .mkdirs import d8n_make_sure_dir_exists
from .path_utils import expand_all
import os


def write_data_to_file(data, filename):
    """ 
        Writes the data to the given filename. 
        If the data did not change, the file is not touched.
    
    """
    if not isinstance(data, str):
        msg = 'Expected "data" to be a string, not %s.' % type(data).__name__
        raise ValueError(msg)
    if len(filename) > 256:
        msg = 'Invalid argument filename: too long. Did you confuse it with data?'
        raise ValueError(msg)
    
    filename = expand_all(filename)
    d8n_make_sure_dir_exists(filename)
    
    if os.path.exists(filename):
        current = open(filename).read()
        if current == data:
            if not 'assets/' in filename:
                logger.debug('already up to date %s' % friendly_path(filename))
            return
         
    with open(filename, 'w') as f:
        f.write(data)
    logger.debug('Written to: %s' % friendly_path(filename))
     
    