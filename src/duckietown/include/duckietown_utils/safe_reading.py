# -*- coding: utf-8 -*-
from contextlib import contextmanager
import gzip
import os
import random
from duckietown_utils.exception_utils import check_isinstance


__all__ = [
    'safe_write',
    'safe_read',
]


def is_gzip_filename(filename):
    check_isinstance(filename, str)
    return '.gz' in filename


@contextmanager
def safe_write(filename, mode='wb', compresslevel=5):
    """ 
        Makes atomic writes by writing to a temp filename. 
        Also if the filename ends in ".gz", writes to a compressed stream.
        Yields a file descriptor.
        
        It is thread safe because it renames the file.
        If there is an error, the file will be removed if it exists.
    """
    check_isinstance(filename, str)
    dirname = os.path.dirname(filename)
    if dirname:
        if not os.path.exists(dirname):
            try:
                os.makedirs(dirname)
            except:
                pass

                # Dont do this!
                # if os.path.exists(filename):
                # os.unlink(filename)
                #     assert not os.path.exists(filename)
                #
    n = random.randint(0, 10000)
    tmp_filename = '%s.tmp.%s.%s' % (filename, os.getpid(), n)
    try:
        if is_gzip_filename(filename):
            fopen = lambda fname, fmode: gzip.open(filename=fname, mode=fmode,
                                                   compresslevel=compresslevel)
        else:
            fopen = open

        with fopen(tmp_filename, mode) as f:
            yield f
        f.close()

        # if os.path.exists(filename):
        # msg = 'Race condition for writing to %r.' % filename
        #             raise Exception(msg)
        #
        # On Unix, if dst exists and is a file, it will be replaced silently
        #  if the user has permission.
        os.rename(tmp_filename, filename)
    except:
        if os.path.exists(tmp_filename):
            os.unlink(tmp_filename)
        if os.path.exists(filename):
            os.unlink(filename)
        raise


@contextmanager
def safe_read(filename, mode='rb'):
    """ 
        If the filename ends in ".gz", reads from a compressed stream.
        Yields a file descriptor.
    """
    try:
        if is_gzip_filename(filename):
            f = gzip.open(filename, mode)
            try:
                yield f
            finally:
                f.close()

        else:
            with open(filename, mode) as f:
                yield f
    except:
        # TODO
        raise
