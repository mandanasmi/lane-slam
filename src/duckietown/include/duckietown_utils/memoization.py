# -*- coding: utf-8 -*-
"""
    Author: Andrea Censi
    
    Origin: snippet from PyMCDP 
"""
 

from decorator import decorator

def memoize_simple(obj):
    cache = obj.cache = {}

    def memoizer(f, *args):
        key = (args)
        if key not in cache:
            cache[key] = f(*args)
        assert key in cache

        try:
            cached = cache[key]
            return cached
        except ImportError: # pragma: no cover  # impossible to test
            del cache[key]
            cache[key] = f(*args)
            return cache[key]

            # print('memoize: %s %d storage' % (obj, len(cache)))

    return decorator(memoizer, obj)
