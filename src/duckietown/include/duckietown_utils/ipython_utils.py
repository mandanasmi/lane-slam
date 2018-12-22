

def ipython_if_guy():
    """ 
    Use like this:
 
        from duckietown_utils.ipython_utils import ipython_if_guy
         
         
        ipython_if_guy()
     
    """
    import getpass
    user = getpass.getuser()
 
    I_am_the_Guy = user in ['rosman']
 
    if I_am_the_Guy:
        import IPython
        IPython.embed()
    else:
        from . import logger
        logger.debug('Username is %r so not starting IPython' % user)

