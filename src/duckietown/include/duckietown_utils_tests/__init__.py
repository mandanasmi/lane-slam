

def jobs_comptests(context):  
    
    from . import hierarchy 
 
    from . import colors
    from . import fuzzy_match_test
    
    from comptests.registrar import jobs_registrar_simple
    jobs_registrar_simple(context)
    
