import time

def format_time_as_YYYY_MM_DD(t):
    return time.strftime('%Y-%m-%d', time.gmtime(t))
    
def format_datetime_as_YYYY_MM_DD(d):
    return d.strftime('%Y-%m-%d')