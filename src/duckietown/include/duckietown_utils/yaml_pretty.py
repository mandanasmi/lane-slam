
# XXX: does not represent None as null, rather as '...\n'
def yaml_load(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    try:
        l = yaml.load(s, Loader=yaml.RoundTripLoader)
    except:
        l = yaml.load(s, Loader=yaml.UnsafeLoader)

    return remove_unicode(l)

def yaml_load_plain(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    l = yaml.load(s, Loader=yaml.UnsafeLoader)
    return remove_unicode(l)

def yaml_dump(s):
    from ruamel import yaml
    res = yaml.dump(s, Dumper=yaml.RoundTripDumper, allow_unicode=False)
    return res

def yaml_dump_pretty(ob):
    from ruamel import yaml
    return yaml.dump(ob, Dumper=yaml.RoundTripDumper)

def remove_unicode(x):

    if isinstance(x, unicode):
        return x.encode('utf8')

    if isinstance(x, dict):
        T = type(x)
        return T([(remove_unicode(k), remove_unicode(v)) for k,v in x.items()])

    if isinstance(x, list):
        T = type(x)
        return T([remove_unicode(_) for _ in x])

    return x

# else:
#     import yaml  # @Reimport
#     def yaml_load(s):
#         return yaml.load(s)
#
#     def yaml_dump(s):
#         return yaml.dump(s)
