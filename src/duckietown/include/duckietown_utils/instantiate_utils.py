import traceback
from .yaml_pretty import yaml_dump


__all__ = ['import_name', 'instantiate']

SemanticMistake = ValueError

def instantiate(function_name, parameters):
    try:
        function = import_name(function_name)
    except ValueError as e:
        msg = 'instantiate(): Cannot find function or constructor %r:\n' % (function_name)
        msg += indent('%s' % (e), '> ')
        raise SemanticMistake(msg)

    try:
        # XXX TypeError is too broad, we should bind the params explicitly
        return function(**parameters)
    except TypeError as e:
#         params = ', '.join(['%s=%r' % (k, v) for (k, v) in parameters.items()])
        msg = 'Could not call this function or instantiate this object:\n'
        msg += '\nConstructor: %s' % function_name
        msg += '\n' + indent(yaml_dump(parameters), '', 'Parameters: ')
        msg += '\n' + indent('%s\n%s' % (e, traceback.format_exc(e)), '> ')
        
#         msg += '\n\n One reason this might be triggered is the presence of pyc files for files that were removed.'
#         msg += '\n\n Use this command to remove them:'
#         msg += '\n\n     make clean-pyc'
#         msg += '\n\n'
        raise SemanticMistake(msg)


def import_name(name):
    ''' 
        Loads the python object with the given name. 
    
        Note that "name" might be "module.module.name" as well.
        
        raise ValueError
    '''
    try:
        return __import__(name, fromlist=['dummy'])
    except ImportError as e:
        # split in (module, name) if we can
        if '.' in name:
            tokens = name.split('.')
            field = tokens[-1]
            module_name = ".".join(tokens[:-1])

            if False:  # previous method
                try:
                    module = __import__(module_name, fromlist=['dummy'])
                except ImportError as e:
                    msg = ('Cannot load %r (tried also with %r):\n' %
                           (name, module_name))
                    msg += '\n' + indent('%s\n%s' % (e, traceback.format_exc(e)), '> ')
                    raise ValueError(msg)

                if not field in module.__dict__:
                    msg = 'No field  %r\n' % (field)
                    msg += ' found in %r.' % (module)
                    raise ValueError(msg)

                return module.__dict__[field]
            else:
                # other method, don't assume that in "M.x", "M" is a module.
                # It could be a class as well, and "x" be a staticmethod.
                try:
                    module = import_name(module_name)
                except ImportError as e:
                    msg = ('Cannot load %r (tried also with %r):\n' %
                           (name, module_name))
                    msg += '\n' + indent('%s\n%s' % (e, traceback.format_exc(e)), '> ')
                    raise ValueError(msg)

                if not field in module.__dict__:
                    msg = 'No field  %r\n' % (field)
                    msg += ' found in %r.' % (module)
                    raise ValueError(msg)

                f = module.__dict__[field]

                # "staticmethod" are not functions but descriptors, we need extra magic
                if isinstance(f, staticmethod):
                    return f.__get__(module, None)
                else:
                    return f

        else:
            msg = 'Cannot import name %r.' % (name)
            msg += '\n' + indent(traceback.format_exc(e), '> ')
            raise ValueError(msg)



def indent(s, prefix, first=None):
    s = str(s)
    assert isinstance(prefix, str)
    lines = s.split('\n')
    if not lines: return ''

    if first is None:
        first = prefix

    m = max(len(prefix), len(first))

    prefix = ' ' * (m - len(prefix)) + prefix
    first = ' ' * (m - len(first)) + first

    # differnet first prefix
    res = ['%s%s' % (prefix, line.rstrip()) for line in lines]
    res[0] = '%s%s' % (first, lines[0].rstrip())
    return '\n'.join(res)
