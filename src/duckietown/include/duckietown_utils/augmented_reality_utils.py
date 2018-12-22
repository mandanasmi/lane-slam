import cv2
import numpy as np
from os.path import basename, expanduser, isfile, join, splitext
from sys import exit

from duckietown_utils import logger, get_duckiefleet_root
from .yaml_pretty import yaml_load


class BaseAugmenter(object):
    '''Base class for doing augmented reality'''
    def __init__(self, veh=''):
        # Robot name
        self.veh = veh
        
        # Masking
        #frustum = mask()
    def callback(self, msg=None):
        pass

    def ground2pixel(self, point):
        '''Transforms point in ground coordinates to point in image
        coordinates using the inverse homography'''
        pass

    def mask(self):
        '''Tests that ground points in the world frame transform to
        the viewing frustum of the camera'''
        pass


    def render_segments(self, image):
        for segment in self.map_data["segments"]:
            pt_x = []
            pt_y = []
            for point in segment["points"]:
                frame, ground_point = self.map_data["points"][point]
                pixel = []
                if frame == 'axle':
                    pixel = self.ground2pixel(ground_point)
                elif frame == 'camera':
                    pixel = ground_point
                else:
                    # logger.info('Unkown reference frame. Using "axle" frame')
                    pixel = self.ground2pixel(ground_point)
                pt_x.append(pixel[0])
                pt_y.append(pixel[1])
            color = segment["color"]
            image = self.draw_segment(image, pt_x, pt_y, color)
        return image

    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red' : ['rgb', [1, 0, 0]],
            'green' : ['rgb', [0, 1, 0]],
            'blue' : ['rgb', [0, 0, 1]],
            'yellow' : ['rgb', [1, 1, 0]],
            'magenta' : ['rgb', [1, 0 ,1]],
            'cyan' : ['rgb', [0, 1, 1]],
            'white' : ['rgb', [1, 1, 1]],
            'black' : ['rgb', [0, 0, 0]]}
        color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]),(pt_x[1], pt_y[1]),(b * 255, g* 255, r * 255), 5)
        return image

#-----------------------------------------------------------------------------#
#                       Augmented reality utils                               #
#-----------------------------------------------------------------------------#

def get_base_name(map_filename):
    return splitext(basename(map_filename))[0]

def load_map(map_filename):
    if not isfile(map_filename):
        print('Map does not exist')
        exit(1)
    map_name = splitext(basename(map_filename))[0]
    with open(map_filename) as f:
        contents = f.read()
        data = yaml_load(contents)
    logger.info('Loaded {} map data'.format(map_name))
    return data

def load_homography(veh):
    path = '/calibrations/camera_'
    filename = get_duckiefleet_root() + path + 'extrinsic/' + veh + '.yaml'
    if not isfile(filename):
        print('Extrinsic calibration for {} does not exist.'.format(veh))
        exit(2)
    with open(filename) as f:
        contents = f.read()
        data = yaml_load(contents)
    logger.info('Loaded homography for {}'.format(veh))
    return np.array(data['homography']).reshape(3,3)

def load_camera_intrinsics(veh):
    path = '/calibrations/camera_'
    filename = get_duckiefleet_root() + path + 'intrinsic/' + veh + '.yaml'
    if not isfile(filename):
        print('Intrinsic calibration for {} does not exist.'.format(veh))
        exit(3)
    with open(filename) as f:
        contents = f.read()
        data = yaml_load(contents)
    intrinsics = {}
    intrinsics['K'] = np.array(data['camera_matrix']['data']).reshape(3, 3)
    intrinsics['D'] = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
    intrinsics['R'] = np.array(data['rectification_matrix']['data']).reshape(3, 3)
    intrinsics['P'] = np.array(data['projection_matrix']['data']).reshape((3,4))
    intrinsics['distortion_model'] = data['distortion_model']
    logger.info('Loaded camera intrinsics for {}'.format(veh))
    return intrinsics

def rectify(image, intrinsics):
    '''Undistort image'''
    height, width, channels = image.shape
    rectified_image = np.zeros(np.shape(image))
    mapx = np.ndarray(shape=(height, width, 1), dtype='float32')
    mapy = np.ndarray(shape=(height, width, 1), dtype='float32')
    mapx, mapy = cv2.initUndistortRectifyMap(intrinsics['K'], intrinsics['D'], intrinsics['R'], intrinsics['P'], (width, height), cv2.CV_32FC1, mapx, mapy)
    return cv2.remap(image, mapx, mapy, cv2.INTER_CUBIC, rectified_image)
