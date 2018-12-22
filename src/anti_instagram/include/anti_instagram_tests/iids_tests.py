from comptests.registrar import run_module_tests, comptest
from anti_instagram_tests.annotations_test import anti_instagram_annotations_test
from easy_logs.cli.require import require_resource
import os
from duckietown_utils import system_cmd_result


@comptest
def iids_tests():
    out_base = 'anti_instagram_annotations_test'
    
    zipname = require_resource('ii-datasets.zip')
    dirname = os.path.dirname(zipname)
    base = os.path.join(dirname, 'ii-datasets')

    cmd = ['unzip', '-o', zipname]
    cwd = dirname
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)
    
    
    if not os.path.exists(base):
        msg = 'Could not find expected unzipped directory:\n   %s' % base
        raise Exception(msg)
    
    anti_instagram_annotations_test(base, out_base)
    
    

if __name__ == '__main__':
    run_module_tests()