# copied from https://github.com/AndreaCensi/system_cmd

from duckietown_utils import logger

import os
import subprocess
import tempfile
from .contracts_ import contract


__all__ = [
    'system_cmd_result',
    'indent_with_label',
]

class Shared():
    p = None 



class CmdResult(object):
    def __init__(self, cwd, cmd, ret, rets, interrupted, stdout, stderr):
        self.cwd = cwd
        self.cmd = cmd
        self.ret = ret
        self.rets = rets
        self.stdout = stdout
        self.stderr = stderr
        self.interrupted = interrupted

    def __str__(self):
        msg = ('The command: %s\n'
               '     in dir: %s\n' % (copyable_cmd(self.cmd), self.cwd))

        if self.interrupted:
            msg += 'Was interrupted by the user\n'
        else:
            msg += 'returned: %s' % self.ret
        if self.rets is not None:
            msg += '\n' + indent(self.rets, 'error>')
        if self.stdout:
            msg += '\n' + indent(self.stdout, 'stdout>')
        if self.stderr:
            msg += '\n' + indent(self.stderr, 'stderr>')
        return msg

    
class CmdException(Exception):
    def __init__(self, cmd_result):
        Exception.__init__(self, str(cmd_result))
        self.res = cmd_result

class CouldNotCallProgram(Exception):
    pass        

@contract(cwd='str', cmd='str|list(str)', env='dict|None')
def system_cmd_result(cwd, cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=False,
                      write_stdin='',
                      capture_keyboard_interrupt=False,
                      env=None): 
    ''' 
        Returns the structure CmdResult; raises CmdException.
        Also OSError are captured.
        KeyboardInterrupt is passed through unless specified
        
        If the program cannot be called at all (OSError for permissions,
        existence, it raises CouldNotCallProgram).
        
        :param write_stdin: A string to write to the process.
    '''
    
    if env is None:
        env = os.environ.copy()

    tmp_stdout = tempfile.TemporaryFile()
    tmp_stderr = tempfile.TemporaryFile()

    ret = None
    rets = None
    interrupted = False

#     if (display_stdout and captured_stdout) or (display_stderr and captured_stderr):        
    
        
    try:
        # stdout = None if display_stdout else 
        stdout = tmp_stdout.fileno()
        # stderr = None if display_stderr else 
        stderr = tmp_stderr.fileno()
        if isinstance(cmd, str):
            cmd = cmd2args(cmd)
        
        assert isinstance(cmd, list)
        if display_stdout or display_stderr:
            logger.info('$ %s' % copyable_cmd(cmd))
        p = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=stdout,
                stderr=stderr,
                bufsize=0,
                cwd=cwd,
                env=env)
#         set_term_function(p)

        if write_stdin != '':
            p.stdin.write(write_stdin)
            p.stdin.flush()

        p.stdin.close()
        p.wait()
        ret = p.returncode
        rets = None
        interrupted = False

    except KeyboardInterrupt:
        logger.debug('Keyboard interrupt for:\n %s' % " ".join(cmd))
        if capture_keyboard_interrupt:
            ret = 100
            interrupted = True
        else:
            raise 
    except OSError as e:
        msg = 'Invalid executable (OSError)'
        msg += '\n      cmd   %s' % cmd[0]
        msg += '\n    errno   %s' % e.errno
        msg += '\n strerror   %s' % e.strerror
        raise CouldNotCallProgram(msg)
        
#         interrupted = False
#         ret = 200
#         rets = str(e)
#         rets = 'OSError: %s' % e

    # remember to go back
    def read_all(f):
        os.lseek(f.fileno(), 0, 0)
        return f.read().strip()

    captured_stdout = read_all(tmp_stdout).strip()
    captured_stderr = read_all(tmp_stderr).strip()
    
    s = ""

    captured_stdout = remove_empty_lines(captured_stdout)
    captured_stderr = remove_empty_lines(captured_stderr)
    if display_stdout and captured_stdout:
        s += indent((captured_stdout), 'stdout>') + '\n'

    if display_stderr and captured_stderr:
        s += indent((captured_stderr), 'stderr>') + '\n'

    if s:
        logger.debug(s)

    res = CmdResult(cwd, cmd, ret, rets, interrupted,
                    stdout=captured_stdout,
                    stderr=captured_stderr)

    if raise_on_error:
        if res.ret != 0:
            raise CmdException(res)

    return res

def remove_empty_lines(s):
    lines = s.split("\n")
    empty = lambda line: len(line.strip()) == 0
    lines = [l for l in lines if not empty(l)]
    return "\n".join(lines)


def cmd2args(s):
    ''' if s is a list, leave it like that; otherwise split()'''
    if isinstance(s, list):
        return s
    elif isinstance(s, str):
        return s.split() 
    else: 
        assert False
    
def wrap(header, s, N=30):
    header = '  ' + header + '  '
    l1 = '-' * N + header + '-' * N
    l2 = '-' * N + '-' * len(header) + '-' * N
    return  l1 + '\n' + s + '\n' + l2

def result_format(cwd, cmd, ret, stdout=None, stderr=None):
    msg = ('Command:\n\t{cmd}\n'
           'in directory:\n\t{cwd}\nfailed with error {ret}').format(
            cwd=cwd, cmd=cmd, ret=ret
           )
    if stdout is not None:
        msg += '\n' + wrap('stdout', stdout)
    if stderr is not None:
        msg += '\n' + wrap('stderr', stderr)
    return msg
#     
# def indent(s, prefix):
#     lines = s.split('\n')
#     lines = ['%s%s' % (prefix, line.rstrip()) for line in lines]
#     return '\n'.join(lines)
#  

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

def indent_with_label(s, first):
    prefix = ' ' * len(first)
    return indent(s, prefix, first)
    
    

@contract(cmds='list(str)')
def copyable_cmd(cmds):
    """ Returns the commands as a copyable string. """
    @contract(x='str')
    def copyable(x):
        if (not ' ' in x) and (not '"' in x) and (not '"' in x):
            return x
        else:
            if '"' in x:
                return "'%s'" % x
            else:
                return '"%s"' % x
            
    return " ".join(map(copyable, cmds))
