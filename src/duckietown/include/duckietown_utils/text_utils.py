
from .contracts_ import contract


# __all__ = ['indent', 'seconds_as_ms']
def indent(s, prefix, first=None):
    s = str(s)
    assert isinstance(prefix, str)
    lines = s.split('\n')
    if not lines: return ''

    if first is None:
        first= prefix

    m = max(len(prefix), len(first))

    prefix = ' ' * (m-len(prefix)) + prefix
    first = ' ' * (m-len(first)) +first

    # differnet first prefix
    res = ['%s%s' % (prefix, line.rstrip()) for line in lines]
    res[0] = '%s%s' % (first, lines[0].rstrip())
    return '\n'.join(res)


def seconds_as_ms(s):
    """ Returns a value in seconds as "XXX ms". """
    if s is None:
        return 'n/a'
    return "%.1f ms" % (s*1000)

def truncate_string_right(s, N, suff=' [..]'):
    if len(s) > N:
        s = s[:N-len(suff)] + suff
    return s

def truncate_string_left(s, N, suff='[..] '):
    if len(s) > N:
        extra = len(s) - N
        s = suff + s[extra+len(suff):]
    return s

import re

escape = re.compile('\x1b\[..?m')
def remove_escapes(s):
    return escape.sub("", s)


def get_length_on_screen(s):
    """ Returns the length of s without the escapes """
    return len(remove_escapes(s))


@contract(table='list', f=str)
def remove_table_field(table, f):
    if not f in table[0]:
        msg = 'Cannot find field %r' % f
        raise ValueError(msg)
    i = table[0].index(f)

    for row in table:
        row.pop(i)

def make_red(s):
    from termcolor import colored
    return  colored(s, 'red')

def make_row_red(row):
    return [ make_red(_) for _ in row]


def format_table_plus(rows, colspacing=1, paginate=25):
    if not rows:
        raise ValueError('Empty table.')
    nfirst = len(rows[0])
    if nfirst == 0:
        raise ValueError('Empty first row.')
    for r in rows:
        if len(r) != nfirst:
            msg = 'Row has len %s while first has length %s.' % (len(r), nfirst)
            raise ValueError(msg)

    # now convert all to string
    rows = [ [str(_) for _ in row] for row in rows]

    # for each column
    def width_cell(s):
        return max(get_length_on_screen(x) for x in s.split('\n'))

    sizes = []
    for col_index in range(len(rows[0])):
        sizes.append(max(width_cell(row[col_index]) for row in rows))

    divider = ['-'*_ for _ in sizes]
    rows.insert(1, divider)

    rows = make_pagination(rows, paginate)
    s = ''
    for row in rows:
        # how many lines do we need?
        nlines = max(num_lines(cell) for cell in row)

        for j in range(nlines):
            for size, cell in zip(sizes, row):
                cellsplit = cell.split('\n')
                if j < len(cellsplit):
                    cellj = cellsplit[j]
                else:
                    cellj = ''
                s += colored_ljust(cellj, size)
                s += ' ' * colspacing
            s += '\n'
    return s

def colored_ljust(string, size):
    a = get_length_on_screen(string)
    remain = size - a
    return string + ' ' * remain

def make_pagination(rows, paginate):
    if len(rows) < paginate:
        return rows
    else:
        header = rows[0]
        divider = rows[1]
        rest = rows[2:]
        pages = []
        while rest:
            n = min(len(rest), paginate)
            pages.append(rest[:n])
            rest = rest[n:]
        result = []
        spaces = [''] * len(header)
        for i, p in enumerate(pages):
            if i != 0:
                result.append(spaces)
            result.append(header)
            result.append(divider)
            result.extend(p)
        return result


def wrap_line_length(x, N):
    res = []
    for l in x.split('\n'):
        while True:
            if not l:
                break
            first, l = l[:N], l[N:]
            res.append(first)
    return "\n".join(res)

def num_lines(s):
    return len(s.split('\n'))


def id_from_basename_pattern(basename, pattern):
    # id_from_basename_pattern('a.b.yaml', '*.b.yaml') => 'a'
    suffix = pattern.replace('*', '')
    ID = basename.replace(suffix, '')
    basename2 = pattern.replace('*', ID)
    assert basename2 == basename, (basename, pattern, ID, basename2)
    return ID


def remove_prefix(s, prefix):
    """
        Removes a prefix from a string.
        Raises ValueError if the prefix is not there.
    """
    if s.startswith(prefix):
        return s[len(prefix):]
    else:
        msg = 'Expected prefix %r in %r' %(prefix, s)
        raise ValueError(msg)



def remove_suffix(s, suffix):
    """
        Removes a prefix from a string.
        Raises ValueError if the prefix is not there.
    """
    if s.endswith(suffix):
        return s[:-len(suffix)]
    else:
        msg = 'Expected suffix %r in %r' %( suffix, s)
        raise ValueError(msg)

def remove_prefix_suffix(s, prefix, suffix):
    s = remove_prefix(s, prefix)
    s = remove_suffix(s, suffix)
    return s

def string_split(s, sub):
    """ Assuming s == a + sub + b, returns a,b """
    if not sub in s:
        msg = 'Substring %r not found in %r.' % (sub, s)
        raise ValueError(msg)
    i = s.index(sub)
    return s[:i], s[i+1:]
