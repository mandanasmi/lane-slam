import urllib2
import socket
from duckietown_utils import logger
from duckietown_utils.memoization import memoize_simple
import ssl
from contracts.utils import indent

use_url = 'http://35.156.29.30/~duckietown/ping'

@memoize_simple
def is_internet_connected(url=use_url, timeout=3):
    """ Use an https server so we know that we are not fooled by
        over-reaching academic network admins """
    socket.setdefaulttimeout(timeout)
    try:
        try:
            urllib2.urlopen(url, timeout=timeout)
        except urllib2.HTTPError as e:
            # we expect 404
            if e.code == 404:
                return True
            else:
                msg = 'Man in the middle attack?'
                msg += '\n\n' + indent(str(e.msg), '> ')
                return False
        return True
    except ssl.CertificateError as e:
        msg = 'Detected proxy; no direct connection available.'
        msg += '\n\n' + indent(e, '  > ')
        logger.warning(msg)
        return False
    except IOError as e:
        logger.warning(e) 
        return False
