from contracts.utils import indent
from duckietown_utils import logger
import os

from .constants import get_duckietown_root
from .exceptions import DTConfigException
from .friendly_path_imp import friendly_path
from .memoization import memoize_simple
from .mkdirs import d8n_make_sure_dir_exists
from .system_cmd_imp import system_cmd_result
from .test_hash import get_md5
from .yaml_pretty import yaml_load


def get_urls_path():
    from .path_utils import get_ros_package_path
    d = get_ros_package_path('easy_logs')
    f = os.path.join(d, 'dropbox.urls.yaml')
    return f

@memoize_simple
def get_dropbox_urls():
    f = get_urls_path()
    if not os.path.exists(f):
        raise DTConfigException(f)
    data = open(f).read()
    
    urls = yaml_load(data)
    
    def sanitize(url):
        if url.endswith('?dl=0'):
            url = url.replace('?dl=0','?dl=1')
        return url
    
    return dict([(k, sanitize(url)) for k, url in urls.items()])


def download_if_not_exist(url, filename):
    if not os.path.exists(filename):
        logger.info('Path does not exist: %s'% filename)
        download_url_to_file(url, filename)
        if not os.path.exists(filename):
            msg = 'I expected download_url_to_file() to raise an error if failed.'
            msg +='\n url: %s' % url
            msg +='\n filename: %s' % filename
            raise AssertionError(msg)
    return filename

def download_url_to_file(url, filename):
    logger.info('Download from %s' % (url))
    tmp = filename + '.tmp_download_file'
    cmd = [
        'wget',
        '-O',
        tmp,
        url
    ]
    d8n_make_sure_dir_exists(tmp)
    res = system_cmd_result(cwd='.', 
                          cmd=cmd,
                          display_stdout=False,
                          display_stderr=False,
                          raise_on_error=True,
                          write_stdin='',
                          capture_keyboard_interrupt=False,
                          env=None)
    
    if not os.path.exists(tmp) and not os.path.exists(filename):
        msg = 'Downloaded file does not exist but wget did not give any error.'
        msg +='\n url: %s' % url
        msg +='\n downloaded to: %s' % tmp
        msg +='\n' + indent(str(res), ' | ')
        d = os.path.dirname(tmp)
        r = system_cmd_result(d, ['ls', '-l'], display_stdout=False,
                          display_stderr=False,
                          raise_on_error=True)
        msg += '\n Contents of the directory:'
        msg += '\n' + indent(str(r.stdout), ' | ')
        raise Exception(msg)
    
    if not os.path.exists(filename):
        os.rename(tmp, filename)
                
    logger.info('-> %s' % friendly_path(filename))

                
def get_file_from_url(url):
    """ 
        Returns a local filename corresponding to the contents of the URL.
        The data is cached in caches/downloads/
    """
    basename = get_md5(url)
    if 'jpg' in url:
        basename += '.jpg'
    filename = os.path.join(get_duckietown_root(), 'caches', 'downloads', basename)
    download_if_not_exist(url, filename)
    return filename


def require_resource(basename, destination=None):
    """ Basename: a file name how it is in urls.yaml 
    
        It returns the URL.
    """
    urls = get_dropbox_urls()
    if not basename in urls:
        msg = 'No URL found for %r.' % basename
        raise Exception(msg)
    else:
        url = urls[basename]
        if destination is None:
            dirname = os.path.join(get_duckietown_root(), 'caches', 'download')
            destination = os.path.join(dirname, basename)
        d8n_make_sure_dir_exists(destination)
        download_if_not_exist(url, destination)
        return destination
    