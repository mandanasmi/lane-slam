'''

Utilities for writing compact file paths.
Copied from the `compmake` project.
    
Author: Andrea Censi

'''

__version__ = '1.0'
import os

# TODO: cache the results?
__all__ = [
    'friendly_path',
]


def friendly_path(path, use_environment=True):
    """ 
        Gets a friendly representation of the given path,
        using relative paths or environment variables
        (if use_environment = True).
    """
    # TODO: send extra rules
    
    options = []

    options.append(os.path.relpath(path, os.getcwd()))

    rules = []
    rules.append(('~', os.path.expanduser('~')))
    rules.append(('.', os.getcwd()))
    rules.append(('.', os.path.realpath(os.getcwd())))

    if use_environment:
        envs = dict(os.environ)
        # remove unwanted 
        for e in list(envs.keys()):
            if 'PWD' in e:
                del envs[e]

        for k, v0 in envs.items():
            if v0:
                for v in [v0, os.path.realpath(v0)]:
                    if v and v[-1] == '/':
                        v = v[:-1]
                    if v[0] == '/':
                        rules.append(('${%s}' % k, v))

    # apply longest first
    rules.sort(key=lambda x: (-len(x[1])))
    path = replace_variables(path, rules)

    options.append(path)

    weight_doubledot = 5

    def score(s):
        # penalize '..' a lot
        s = s.replace('..', '*' * weight_doubledot)
        return len(s)

    options.sort(key=score) 
    result = options[0]

    # print('Converted %s  => %s' % (original, result))

    return result

def replace_variables(path, rules):
    for k, v in rules:
        if path.startswith(v):
            # print("  applied %s => %s" % (v, k))
            path = path.replace(v, k)
    return path


