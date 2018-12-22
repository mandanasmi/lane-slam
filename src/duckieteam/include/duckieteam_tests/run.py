from comptests.registrar import run_module_tests, comptest

from duckietown_utils import system_cmd_result


@comptest
def run1():
    cmd = ['rosrun', 'duckieteam', 'create-machines', '--print']
    cwd = '.'
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@comptest
def run2():
    tmpfile = '/tmp/tmp'
    cmd = ['rosrun', 'duckieteam', 'create-roster', '--roster', tmpfile]
    cwd = '.'
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)
    
@comptest
def run():
    cmd = ['rosrun', 'duckieteam', 'create-roster']
    cwd = '.'
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)
    
if __name__ == '__main__':
    run_module_tests()
    
