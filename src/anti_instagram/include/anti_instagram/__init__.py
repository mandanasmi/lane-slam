import logging
logging.basicConfig()

logger = logging.getLogger('anti_instagram')
logger.setLevel(logging.DEBUG)

from .AntiInstagram import *
from .kmeans import *
from .utils import *
from .scale_and_shift import *
