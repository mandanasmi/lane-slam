from collections import OrderedDict, namedtuple
from comptests.registrar import comptest, run_module_tests
from contracts.utils import check_isinstance
import re

from duckietown_utils.fuzzy import fuzzy_match, parse_match_spec,\
    slice_regexp
from duckietown_utils.instantiate_utils import indent


def expect(data, query, result_keys, filters=None):
    result_keys = list(result_keys)
    check_isinstance(data, OrderedDict)
    
    if True:
        spec = parse_match_spec(query, filters=filters)
        print '-----'
        print 'Query: %s' % query
        print indent(spec, '', 'Spec: ')
        
    res = fuzzy_match(query, data, filters=filters)
    check_isinstance(res, OrderedDict)
    
    if list(res) != result_keys:
        msg = 'Error:'
        msg += '\n Data: %s' % data
        msg += '\n Query: %s' % query
        msg += '\n Result: %s' % list(res)
        msg += '\n Expect: %s' % list(result_keys)
        raise Exception(msg)
    
@comptest
def matches():
    Species = namedtuple('Species', 'name size')
    data = OrderedDict([
        ('one',  Species('horse', 'large')),
        ('two',  Species('dog', 'medium')),
        ('three',  Species('cat', 'medium')) 
    ])
    
    expect(data, 'all', ['one', 'two', 'three'])
    expect(data, '*', ['one', 'two', 'three'])
    expect(data, 'one', ['one'])
    expect(data, 'two+one', ['two', 'one'])
    expect(data, '*o', ['two'])
    expect(data, 'o*', ['one'])
    
    expect(data, 'all/first', ['one'])
    expect(data, '/first', ['one'])

@comptest
def matches_tags():
    Species = namedtuple('Species', 'name size weight')
    data = OrderedDict([
        ('jeb',  Species('A big horse', 'large', 200)),
        ('fuffy',  Species('A medium dog', 'medium', 50)),
        ('ronny',  Species('A medium cat', 'medium', 30)), 
    ])
    
    expect(data, '*', ['jeb', 'fuffy', 'ronny'])

    expect(data, 'name:A big horse', ['jeb'])
    expect(data, 'name:*medium*', ['fuffy', 'ronny'])
    expect(data, 'name:*medium*,weight:>40', ['fuffy'])
    
    expect(data, 'name:*medium*,weight:<40', ['ronny'])
    expect(data, 'fuffy+ronny', ['fuffy','ronny'])
    expect(data, 'name:*dog', ['fuffy'])
    expect(data, 'name:*cat', ['ronny'])
    expect(data, 'name:*dog+name:*cat', ['fuffy','ronny'])
    
@comptest
def specs1():
    parse_match_spec('ciao')
    parse_match_spec('ciao+no')


@comptest
def my_filter():
   
    rc = re.compile(slice_regexp)
    m = rc.search('[1:2:3]')
    assert m.group('a') == '1'
    assert m.group('b') == '2'
    assert m.group('c') == '3'
   
    Species = namedtuple('Species', 'name size weight')
    data = OrderedDict([
        ('jeb',  Species('A big horse', 'large', 200)),
        ('fuffy',  Species('A medium dog', 'medium', 50)),
        ('ronny',  Species('A medium cat', 'medium', 30)), 
    ])
    expect(data, 'all', ['jeb','fuffy','ronny'])
    
    expect(data, 'all/[0]', ['jeb'])
    expect(data, 'all/[0:]', ['jeb','fuffy','ronny'])
    expect(data, 'all/[0:1]', ['jeb'])
    expect(data, 'all/[:-1]', ['jeb', 'fuffy'])
    expect(data, 'all/[::1]', ['jeb', 'fuffy','ronny'])
    expect(data, 'all/[::2]', ['jeb',  'ronny'])
    expect(data, 'all/[0:3]', ['jeb', 'fuffy', 'ronny'])
    
    
    res = fuzzy_match('all/shuffle', data)
    assert len(res)==3
    
if __name__ == '__main__':
    run_module_tests()
    
    