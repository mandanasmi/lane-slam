import os
import subprocess

from .caching import get_cached
from .yaml_pretty import yaml_load


__all__ = ['rosbag_info', 'rosbag_info_cached']

def rosbag_info_cached(filename):
    def f():
        return rosbag_info(filename)
    basename = os.path.basename(filename)
    cache_name = 'rosbag_info/' + basename
    return get_cached(cache_name, f, quiet=True)

def rosbag_info(bag): 
    stdout = subprocess.Popen(['rosbag', 'info', '--yaml', bag],
                              stdout=subprocess.PIPE).communicate()[0]
#     try:
    info_dict = yaml_load(stdout)
#     except:
#         logger.error('Could not parse yaml:\n%s' % indent(stdout, '| '))
#         raise
    return info_dict

def which_robot(bag):
    import re
    pattern  = r'/(\w+)/camera_node/image/compressed'
    
    topics = list(bag.get_type_and_topic_info()[1].keys())

    for topic in topics:
        m = re.match(pattern, topic)
        if m:
            vehicle = m.group(1)
            return vehicle
    msg = 'Could not find a topic matching %s' % pattern
    raise ValueError(msg)

def get_image_topic(bag):
    """ Returns the name of the topic for the main camera """
    topics = bag.get_type_and_topic_info()[1].keys()
    for t in topics:
        if 'camera_node/image/compressed' in t:
            return t
    msg = 'Cannot find the topic: %s' % topics
    raise ValueError(msg)

def d8n_get_all_images_topic(bag_filename):
    """ 
        Returns the (name, type) of all topics that look like images. 
    """
    import rosbag  # @UnresolvedImport
    bag = rosbag.Bag(bag_filename)
    return d8n_get_all_images_topic_bag(bag)

def d8n_get_all_images_topic_bag(bag):
    """ 
        Returns the (name, type) of all topics that look like images. 
    """
    tat = bag.get_type_and_topic_info()
    consider_images = [
        'sensor_msgs/Image',
        'sensor_msgs/CompressedImage',
    ]
    all_types = set()
    found = []
    topics = tat.topics
    for t,v in topics.items():
        msg_type = v.msg_type
        all_types.add(msg_type)
        _message_count = v.message_count
        if msg_type in consider_images:

            # quick fix: ignore image_raw if we have image_compressed version
            if 'raw' in t:
                other = t.replace('raw', 'compressed')

                if other in topics:
                    continue
            found.append((t,msg_type))
    return found

