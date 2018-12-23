from abc import ABCMeta, abstractmethod
from collections import namedtuple

FAMILY_LANE_FILTER = 'lane_filter'

#Detections = namedtuple('Detections', 
#                        ['lines','normals','area','centers'])


class LaneFilterInterface():
    __metaclass__ = ABCMeta

#    @abstractmethod
#    def setImage(self, bgr):
#        pass
#
#    @abstractmethod
#    def detectLines(self, color):
#        """ Returns a tuple of class Detections """


