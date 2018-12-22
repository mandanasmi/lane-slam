# from . import logger
from collections import defaultdict
# from contracts import contract
import fnmatch
import os

__all__ = [
    'locate_files',
]


# @contract(returns='list(str)', directory='str',
#           pattern='str', followlinks='bool')
def locate_files(directory, pattern, followlinks=True, alsodirs=False):
    # print('locate_files %r %r' % (directory, pattern))
    filenames = []

    for root, dirs, files in os.walk(directory, followlinks=followlinks):
        if alsodirs:
            for f in dirs:
                if fnmatch.fnmatch(f, pattern):
                    filename = os.path.join(root, f)
                    filenames.append(filename)

        for f in files:
            if fnmatch.fnmatch(f, pattern):
                filename = os.path.join(root, f)
                filenames.append(filename)

    real2norm = defaultdict(lambda: [])
    for norm in filenames:
        real = os.path.realpath(norm)
        real2norm[real].append(norm)
        # print('%s -> %s' % (real, norm))

    for k, v in real2norm.items():
        if len(v) > 1:
            msg = 'In directory:\n\t%s\n' % directory
            msg += 'I found %d paths that refer to the same file:\n'
            for n in v:
                msg += '\t%s\n' % n
            msg += 'refer to the same file:\n\t%s\n' % k
            msg += 'I will silently eliminate redundancies.'
            # logger.warning(v)

    return list(real2norm.keys())

