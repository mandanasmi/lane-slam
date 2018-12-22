import os

from duckietown_utils import logger

from .friendly_path_imp import friendly_path
from .path_utils import expand_all
from .safe_pickling import safe_pickle_load, safe_pickle_dump


__all__ = [
    'get_cached',
]

def get_cached(cache_name, f, quiet='not-given'):
    """
        Caches the result of f() in a file called
            ${DUCKIETOWN_ROOT}/caches/![name].cache.pickle
    """


    cache = '${DUCKIETOWN_ROOT}/caches/%s.cache.pickle' % cache_name
    cache = expand_all(cache)

    if quiet == 'not-given':
        should_be_quiet = False
    else:
        should_be_quiet = quiet

    if os.path.exists(cache):

        if not should_be_quiet:
            logger.info('Using cache %s' % friendly_path(cache))

        try:
            return safe_pickle_load(cache)
        except:
            msg = 'Removing cache that I cannot read: %s' % friendly_path(cache)
            logger.error(msg)
            os.unlink(cache)


    ob = f()
    if not should_be_quiet:
        logger.info('Writing to cache %s' % friendly_path(cache))
    try:
        os.makedirs(os.path.dirname(cache))
    except:
        pass

    safe_pickle_dump(ob, cache)

#         with open(cache, 'w') as f:
#             cPickle.dump(ob, f)
#
#
    return ob
