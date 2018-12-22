
from comptests.registrar import comptest, run_module_tests

from termcolor import colored

from duckietown_utils.text_utils import (get_length_on_screen, format_table_plus, 
                                         remove_escapes, make_row_red)


@comptest
def test_color_sizes():
    s1 = 'one'
    
    s2 = colored(s1, 'magenta')
     
#     print(s1.__repr__())
#     print(s2.__repr__())
    
    l1 = get_length_on_screen(s1)
    l2 = get_length_on_screen(s2)
#     print l1, l2
    assert l1 == l2
    
    
def get_test_table():
    table = []
    for i in range(3):
        row = []
        for j in range(5):
            row.append('*' * i * j)
        table.append(row)
    return table

@comptest
def test_table():
    
    table = get_test_table()
    r1 = format_table_plus(table)
    
    table[1] = make_row_red(table[1])
    r2 = format_table_plus(table)
    
    #print r1
    #print r2
    r2e = remove_escapes(r2)
    #print r2e
    assert r1 == r2e
    

if __name__ == '__main__':
    run_module_tests()
    