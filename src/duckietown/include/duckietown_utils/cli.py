import os

from quickapp.quick_app import QuickApp
from quickapp.quick_app_base import QuickAppBase

from duckietown_utils.exceptions import DTUserError, wrap_script_entry_point
from easy_logs.logs_db import get_easy_logs_db_cached_if_possible,\
    get_easy_logs_db_cloud, get_easy_logs_db_fresh


class D8App(QuickAppBase):
    
    def get_from_args_or_env(self, argname, envname):
        """ Gets either the argumnent or the environment variable."""
        options = [getattr(self.options, argname), os.environ.get(envname, None)]
        options = [_ for _ in options if _ and _.strip()]
        if not options:
            msg = ('Either provide command line argument --%s or environment variable %s.' %
                    (argname, envname))
            raise DTUserError(msg)     
        return options[0]
    
def d8app_run(App):
    main = App.get_sys_main()
    wrap_script_entry_point(main)
      
class D8AppWithLogs(D8App):
    """ 
        An app that works with a log database. 
    
        Adds the options --cache and --cloud, for working with logs. 
    """

    def define_program_options(self, params):
        if hasattr(self, '_define_options_compmake'):
            self._define_options_compmake(params)
        self._define_my_options(params)
        self.define_options(params)

    def _define_my_options(self, params):
        g = "Options regarding the logs database"
        params.add_flag('cache', help="Use local log cache.", group=g) 
        params.add_flag('cloud', help="Use cloud DB", group=g)
        self._db = None
    
    def get_easy_logs_db(self):
        if self._db is not None:
            return self._db
        
        use_cache = self.options.cache 
        use_cloud = self.options.cloud
        if use_cache and use_cloud:
            msg = 'Cannot use --cache and --cloud together.'
            raise DTUserError(msg)
        
        if use_cache:
            db = get_easy_logs_db_cached_if_possible()
        elif use_cloud:
            db = get_easy_logs_db_cloud()
        else:
            db = get_easy_logs_db_fresh()
        
        self._db = db
        return db


    
class D8AppWithJobs(D8App, QuickApp):
    pass