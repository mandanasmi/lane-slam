# copied from compmake

import os

__all__ = [
    'd8n_make_sure_dir_exists',
    'd8n_mkdirs_thread_safe',
]


def d8n_make_sure_dir_exists(filename):
    """ 
        Makes sure that the path to file exists, by creating directories. 
        
    """
    dirname = os.path.dirname(filename)
    
    # dir == '' for current dir
    if dirname != '' and not os.path.exists(dirname):
        d8n_mkdirs_thread_safe(dirname)
        
def d8n_mkdirs_thread_safe(dst):
    """ 
        Make directories leading to 'dst' if they don't exist yet.
        
        This version is thread safe.
        
    """
    if dst == '' or os.path.exists(dst):
        return
    head, _ = os.path.split(dst)
    if os.sep == ':' and not ':' in head:
        head += ':'
    d8n_mkdirs_thread_safe(head)
    try:
        mode = 511  # 0777 in octal
        os.mkdir(dst, mode)
    except OSError as err:
        if err.errno != 17:  # file exists
            raise

