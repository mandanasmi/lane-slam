from .kmeans import getparameters2, identifyColors, runKMeans
from .scale_and_shift import scaleandshift
from anti_instagram.kmeans import CENTERS, CENTERS2
import numpy as np
from duckietown_utils import logger

def calculate_transform(image):
    """
        Returns tuple (bool, float, parameters)
        
        success, health, parameters
        
        parameters['scale']
        parameters['shift']
    """
    centers4 = CENTERS2
    trained4, counter4,score4 = runKMeans(image, num_colors=4, init=centers4)
    trained4 = trained4[[0,2,3],:]
    counter4 = [counter4[0],counter4[2],counter4[3]]
    centers4 = centers4[[0,2,3],:]
    centers3 = CENTERS
    trained3, counter3,score3 = runKMeans(image, num_colors=3, init=centers3)
    decision34=(score3+3e7)>score4;
    if (decision34):
        logger.info("picked 3 colors")
        trained=trained3
        counter=counter3
        centers=centers3
    else:
        logger.info("picked 4 colors")
        trained=trained4
        counter=counter4
        centers=centers4
 

    mapping = identifyColors(trained, centers)
    r, g, b, cost = getparameters2(mapping, trained, counter, centers)
    
    if r[0][0] == 0.0:
        # XXX: not sure what this is supposed to be
        return False, 0.0, None

    scale = np.array([r[0][0][0],g[0][0][0],b[0][0][0]])
    shift = np.array([r[1][0], g[1][0],b[1][0]])
    
    eps = np.finfo('double').eps
    health = 1.0 / (cost + eps)
    
    parameters = dict(scale=scale, shift=shift)
    
    return True, float(health), parameters

#     # Estimates the scale and shift over multiple frame via an IIR filter with preference towards low-cost frames
#     IIR_weight=1000/(10000+cost)
#     #logger.info("cost = %f, IIR_weight = %f" % (cost, IIR_weight))
#     # self.scale = [r[0][0][0],g[0][0][0],b[0][0][0]]
#     # self.shift = [r[1][0], g[1][0],b[1][0]]
#     deltascale = np.array([r[0][0][0],g[0][0][0],b[0][0][0]])
#     deltashift = np.array([r[1][0], g[1][0],b[1][0]])
#     if testframe:
#         self.scale = deltascale
#         self.shift = deltashift
#     else:
#         self.scale = (self.scale+deltascale*IIR_weight)/(1+IIR_weight)
#         self.shift = (self.shift+deltashift*IIR_weight)/(1+IIR_weight)

class ScaleAndShift():
    """ Represents the transformation """
    
    def __init__(self, scale, shift):
        self.scale = scale
        self.shift = shift
    
    def __call__(self, image):
        corrected_image = scaleandshift(image, self.scale, self.shift)
        return corrected_image
    
    @staticmethod
    def identity():
        return ScaleAndShift([1.0,1.0,1.0], [0.0,0.0,0.0])



class AntiInstagram():

    def __init__(self):
        self.scale = [1.0, 1.0, 1.0]
        self.shift = [0.0, 0.0, 0.0]
        self.health = 0
    
    def applyTransform(self, image):
        corrected_image = scaleandshift(image, self.scale, self.shift)
        return corrected_image
    
    def calculateTransform(self, image, testframe=False):
        success, self.health, parameters = calculate_transform(image)
        if not success:
            raise Exception('calculate_transform failed')
        self.scale = parameters['scale']
        self.shift = parameters['shift']
    
    def calculateHealth(self):
        return self.health
