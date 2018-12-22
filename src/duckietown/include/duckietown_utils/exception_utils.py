""" 
    All of this is copied from [PyContracts](todo).
    
"""

import sys

from .text_utils import indent


def _get_str(x, informal):
    from contracts.interface import describe_value_multiline
    if informal:
        s = str(x)
    else:
        s = describe_value_multiline(x)
    return s


def format_list_long(l, informal=False):
    """
        - My 
          first
        - Second
    """
    res = ""
    for i, value in enumerate(l):
        prefix = '- '
        if i > 0:
            res += '\n'
        s = _get_str(value, informal)
        res += indent(s, ' ', first=prefix)
    return res

def format_obs(d, informal=False):
    """ Shows objects values and typed for the given dictionary """
    if not d:
        return str(d)

    maxlen = 0
    for name in d:
        maxlen = max(len(name), maxlen)

    def pad(pre):
        return ' ' * (maxlen-len(pre)) + pre

    res = ''

    S = sorted(d)
    for i, name in enumerate(S):
        value = d[name]
        prefix = pad('%s: ' % name)
        if i > 0:
            res += '\n'

        s = _get_str(value, informal)

        res += indent(s, ' ', first=prefix)

    return res


def raise_wrapped(etype, e, msg, compact=False, exc=None, **kwargs):
    """ Raises an exception of type etype by wrapping
        another exception "e" with its backtrace and adding
        the objects in kwargs as formatted by format_obs.
        
        if compact = False, write the whole traceback, otherwise just str(e).
    
        exc = output of sys.exc_info()
    """
    
    e = raise_wrapped_make(etype, e, msg, compact=compact, **kwargs)
    
    if exc is not None:
        _, _, trace = exc
        raise etype, e.args, trace
    else:
        raise e
    
def raise_wrapped_make(etype, e, msg, compact=False, **kwargs):
    """ Constructs the exception to be thrown by raise_wrapped() """
    assert isinstance(e, BaseException), type(e)
    assert isinstance(msg, (str, unicode)), type(msg)
    s = msg
    if kwargs:
        s += '\n' + format_obs(kwargs)

    import sys
    if sys.version_info[0] >= 3:
        es = str(e)
    else:
        if compact:
            es = str(e)
        else:
            import traceback
            es = traceback.format_exc(e)

    s += '\n' + indent(es.strip(), '| ')

    return etype(s)

def raise_desc(etype, msg, args_first=False, **kwargs):
    """
    
        Example:
            raise_desc(ValueError, "I don't know", a=a, b=b)
    """
    assert isinstance(msg, str), type(msg)
    s1 = msg
    if kwargs:
        s2 = format_obs(kwargs)
    else:
        s2 = ""

    if args_first:
        s = s2 + "\n" + s1
    else:
        s = s1 + "\n" + s2

    raise etype(s)



# A couple of functions for pretty errors
def aslist(x):
    if isinstance(x, dict):
        x = list(x.keys())
    if x:
        return ", ".join([e.__repr__() for e in sorted(x)])
    else:
        return "<empty>"


def raise_x_not_found(what, x, iterable, exception=ValueError):
    msg = x_not_found(what, x, iterable)
    raise exception(msg)

def x_not_found(what, x, iterable):
    ''' Shortcut for creating pretty error messages. '''
    # TODO: add guess in case of typos
    options = aslist(iterable)
    
    return ('Could not find %s %r. I know the elements: %s.' %
            (what, x, options))


def check_is_in(what, x, iterable, exception=ValueError):
    if not x in iterable:
        raise_x_not_found(what, x, iterable, exception) 


def check_isinstance(ob, expected, **kwargs):
    if not isinstance(ob, expected):
        kwargs['object'] = ob
        raise_type_mismatch(ob, expected, **kwargs)
        
def raise_type_mismatch(ob, expected, **kwargs):
    """ Raises an exception concerning ob having the wrong type. """
    e = 'Object not of expected type:'
    e +='\n  expected: %s' % str(expected)
    e +='\n  obtained: %s' % str(type(ob))
    e += '\n' + indent(format_obs(kwargs), ' ')
    raise ValueError(e)


def describe_type(x):
    """ Returns a friendly description of the type of x. """
    
    inPy2 = sys.version_info[0] == 2
    if inPy2:
        from types import ClassType
        
    if inPy2 and isinstance(x, ClassType):
        class_name = '(old-style class) %s' % x
    else:
        if hasattr(x, '__class__'):
            c = x.__class__
            if hasattr(x, '__name__'):
                class_name = '%s' % c.__name__
            else:
                class_name = str(c)
        else:
            # for extension classes (spmatrix)
            class_name = str(type(x))

    return class_name


def describe_value(x, clip=80):
    """ Describes an object, for use in the error messages.
        Short description, no multiline.
    """
    if hasattr(x, 'shape') and hasattr(x, 'dtype'):
        shape_desc = 'x'.join(str(i) for i in x.shape)
        desc = 'array[%r](%s) ' % (shape_desc, x.dtype)
        final = desc + clipped_repr(x, clip - len(desc))
        return remove_newlines(final)
    else:
        class_name = describe_type(x)
        desc = 'Instance of %s: ' % class_name
        final = desc + clipped_repr(x, clip - len(desc))
        return remove_newlines(final)

def clipped_repr(x, clip):
    s = "{0!r}".format(x)
    if len(s) > clip:
        clip_tag = '... [clip]'
        cut = clip - len(clip_tag)
        s = "%s%s" % (s[:cut], clip_tag)
    return s


# TODO: add checks for these functions


def remove_newlines(s):
    return s.replace('\n', ' ')
