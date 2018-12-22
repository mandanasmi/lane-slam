import os

from duckietown_utils.bag_info import get_image_topic

from . import logger
from .bag_reading import BagReadProxy
from .expand_variables import expand_environment
from .image_conversions import numpy_from_ros_compressed

#
# __all__ = [
#     'd8n_read_images_interval',
#     'd8n_read_all_images',
#     'd8n_get_all_images_topic',
# ]
def d8n_read_images_interval(filename, t0, t1):
    """
        Reads all the RGB data from the bag,
        in the interval [t0, t1], where t0 = 0 indicates
        the first image.

    """
    data = d8n_read_all_images(filename, t0, t1)
    logger.info('Read %d images from %s.' % (len(data), filename))
    timestamps = data['timestamp']
    # normalize timestamps
    first = data['timestamp'][0]
    timestamps -= first
    logger.info('Sequence has length %.2f seconds.' % timestamps[-1])
    return data

def d8n_read_all_images(filename, t0=None, t1=None):
    """
        Raises a ValueError if not data could be read.

        Returns a numpy array.

            data = d8n_read_all_images(bag)

            print data.shape # (928,)
            print data.dtype # [('timestamp', '<f8'), ('rgb', 'u1', (480, 640, 3))]
    """

    filename = expand_environment(filename)
    if not os.path.exists(filename):
        msg = 'File does not exist: %r' % filename
        raise ValueError(msg)
    import rosbag  # @UnresolvedImport
    bag = rosbag.Bag(filename)
    topic = get_image_topic(bag)
    bag_proxy = BagReadProxy(bag, t0, t1)
    # FIXME: this is wrong
    res = d8n_read_all_images_from_bag(bag_proxy, topic, t0=t0,t1=t1)
    return res

def d8n_read_all_images_from_bag(bag, topic0, max_images=None):
    import numpy as np

    nfound = bag.get_message_count(topic_filters=topic0)
    logger.info('Found %d images for %s' % (nfound, topic0))    
    logger.debug('max_images = %d ' % (max_images))    
        
    data = []
    first_timestamp = None

    if max_images is None:
        interval = None
    else:
        interval = int(np.ceil(nfound / max_images))
        if interval == 0:
            interval = 1
        logger.info('Read %s images; interval = %d' % (nfound, interval))
           
    for j, (topic, msg, t) in enumerate(bag.read_messages()):
        if topic == topic0:
            float_time = t.to_sec()
            if first_timestamp is None:
                first_timestamp = float_time
            
            if interval is not None:
                add = (j % interval == 0)
                if not add:
                    continue
                
            rgb = numpy_from_ros_compressed(msg)

            data.append({'timestamp': float_time, 'rgb': rgb})

            if j % 10 == 0:
                logger.debug('Read %d images from topic %s' % (j, topic))

    logger.info('Returned %d images' % len(data))
    if not data:
        raise ValueError('no data found')

    H, W, _ = rgb.shape  # (480, 640, 3)
    logger.info('Detected image shape: %s x %s' % (W, H))
    n = len(data)
    
    dtype = [
        ('timestamp', 'float'),
        ('rgb', 'uint8', (H, W, 3)),
    ]

    x = np.zeros((n,), dtype=dtype)
    logger.info('dtype = %s' % x.dtype)

    for i, v in enumerate(data):
#         print(i, v['rgb'].shape)
        x[i]['timestamp'] = v['timestamp']
        x[i]['rgb'][:] = v['rgb']

    logger.info('dtype = %s, shape=%s' % (x.dtype, x.shape))
    return x
