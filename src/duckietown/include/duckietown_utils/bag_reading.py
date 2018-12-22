from duckietown_utils import logger
import time

import numpy as np

from .exceptions import DTBadData


class BagReadProxy():
    
    def __init__(self, bag, t0, t1):
        """
            t0, t1 are relative times to the bag start
            
            They can be None, in which case they are unbounded.
        """
        bag_absolute_t0 = bag.get_start_time()
        bag_absolute_t1 = bag.get_end_time()
        if t0 is not None:
            read_from = bag_absolute_t0 + t0
        else:
            read_from = bag_absolute_t0
        if t1 is not None:
            read_to = bag_absolute_t0 + t1
        else:
            read_to = bag_absolute_t1
            
        self.read_from_absolute = read_from
        self.read_to_absolute = read_to
        self.bag = bag
        
        seen = read_to -read_from
        total = bag_absolute_t1 - bag_absolute_t0
        self.fraction = seen / total
        
    def get_type_and_topic_info(self):
        return self.bag.get_type_and_topic_info()
    
    def get_start_time(self):
        return self.read_from_absolute 
    def get_end_time(self):
        return self.read_to_absolute 
            
    def get_message_count(self, topic_filters=None):
        """ Returns approximate message count, compensating with ratio. """
        n = self.bag.get_message_count(topic_filters)
        n1 = int(np.ceil(self.fraction * n))
        print('n = %s  fraction = %s  n1 = %s' % (n, self.fraction, n1) )
        return n1 
    
    def read_messages(self, *args, **kwargs):
        for topic, msg, _t in self.bag.read_messages(*args,**kwargs):
            t = _t.to_sec()
            if t < self.read_from_absolute:
                continue
            if t > self.read_to_absolute:
                break
            yield topic, msg, _t
            
    def close(self):
        self.bag.close()
     

def d8n_bag_read_with_progress(bag, topic):
    bag_t0 = bag.get_start_time()
    bag_t1 = bag.get_end_time()
    length = bag_t1 - bag_t0  
    n = 0
    msg_time0 = None
    # write a message every once in a while 
    INTERVAL = 1
    first = last = time.time()
    for topic, msg, msg_time in bag.read_messages(topics=[topic]):
        # compute progess
        msg_time = msg_time.to_sec()
        if msg_time0 is None:
            msg_time0 = msg_time
        progress = float(msg_time-msg_time0) / length
        
        # current time
        n += 1 
        t = time.time()
        if t - last > INTERVAL:
            last =  t
            fps =  n / (t-first) 
            logger.debug('%6d  %4.1f%%  %5.1f fps' % (n, progress * 100, fps))
        yield msg
    if n == 0:
        s = 'Could not find any message for topic %r.' % topic
        raise DTBadData(s)
    
    fps = n / (time.time() - first)
    logger.debug('Read %d messages for %s. Processing time: %.1f fps.' % (n, topic, fps))
    bag.close()
    
