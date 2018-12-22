import numpy as np
import cv2

from .line_detector_interface import Detections, LineDetectorInterface

from duckietown_utils.parameters import Configurable

class LineDetector2Dense(Configurable, LineDetectorInterface):
    def __init__(self, configuration):
        # Images to be processed
        self.bgr = np.empty(0)
        self.hsv = np.empty(0)
        self.edges = np.empty(0)

        param_names = [

            'hsv_white1',
            'hsv_white2',
            'hsv_yellow1',
            'hsv_yellow2',
            'hsv_red1',
            'hsv_red2',
            'hsv_red3',
            'hsv_red4',

            'dilation_kernel_size',
            'canny_thresholds',
            'sobel_threshold',
        ]

        Configurable.__init__(self, param_names, configuration)

    def _colorFilter(self, color):
        # threshold colors in HSV space
        if color == 'white':
            bw = cv2.inRange(self.hsv, self.hsv_white1, self.hsv_white2)
        elif color == 'yellow':
            bw = cv2.inRange(self.hsv, self.hsv_yellow1, self.hsv_yellow2)
        elif color == 'red':
            bw1 = cv2.inRange(self.hsv, self.hsv_red1, self.hsv_red2)
            bw2 = cv2.inRange(self.hsv, self.hsv_red3, self.hsv_red4)
            bw = cv2.bitwise_or(bw1, bw2)
        else:
            raise Exception('Error: Undefined color strings...')

        # binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.dilation_kernel_size, self.dilation_kernel_size))

        # refine edge for certain color
        edge_color = cv2.bitwise_and(cv2.dilate(bw, kernel), self.edges)

        return bw, edge_color

    def _lineFilter(self, bw, edge_color):
        # find gradient of the bw image
        grad_x = -cv2.Sobel(bw/255, cv2.CV_32F, 1, 0, ksize=5)
        grad_y = -cv2.Sobel(bw/255, cv2.CV_32F, 0, 1, ksize=5)
        grad_x *= (edge_color == 255)
        grad_y *= (edge_color == 255)

        # compute gradient and thresholding
        grad = np.sqrt(grad_x**2 + grad_y**2)
        roi = (grad>self.sobel_threshold)

        #print np.unique(grad)
        #print np.sum(roi)

        # turn into a list of points and normals
        roi_y, roi_x = np.nonzero(roi)
        centers = np.vstack((roi_x, roi_y)).transpose()
        normals = np.vstack((grad_x[roi], grad_y[roi])).transpose()
        normals /= np.sqrt(np.sum(normals**2, axis=1, keepdims=True))

        lines = self._synthesizeLines(centers, normals)

        return lines, normals, centers

    def _findEdge(self, gray):
        edges = cv2.Canny(gray, self.canny_thresholds[0], self.canny_thresholds[1], apertureSize = 3)
        return edges

    def _checkBounds(self, val, bound):
        val[val<0]=0
        val[val>=bound]=bound-1
        return val

    def _synthesizeLines(self, centers, normals):
        lines = []
        if len(centers)>0:
            x1 = (centers[:,0:1] + normals[:, 1:2] * 6.).astype('int')
            y1 = (centers[:,1:2] - normals[:, 0:1] * 6.).astype('int')
            x2 = (centers[:,0:1] - normals[:, 1:2] * 6.).astype('int')
            y2 = (centers[:,1:2] + normals[:, 0:1] * 6.).astype('int')
            x1 = self._checkBounds(x1, self.bgr.shape[1])
            y1 = self._checkBounds(y1, self.bgr.shape[0])
            x2 = self._checkBounds(x2, self.bgr.shape[1])
            y2 = self._checkBounds(y2, self.bgr.shape[0])
            lines = np.hstack([x1, y1, x2, y2])
        return lines

    def detectLines(self, color):
        bw, edge_color = self._colorFilter(color)
        lines, normals, centers = self._lineFilter(bw, edge_color)
        return Detections(lines=lines, normals=normals, area=bw, centers=centers)

    def setImage(self, bgr):
        self.bgr = np.copy(bgr)
        self.hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.edges = self._findEdge(self.bgr)

    def getImage(self):
        return self.bgr


