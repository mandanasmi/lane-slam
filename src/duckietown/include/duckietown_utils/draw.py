import cv2
import numpy as np

from duckietown_utils import load_homography, load_map


class Draw():
    '''class for drawing projected line segments'''
    def __init__(self, robot_name='', map_file=''):
        # Set robot name
        self.robot_name = robot_name

        # Load map
        self.map_data = load_map(map_file)

        # Load homography
        self.H = load_homography(self.robot_name)

    def mask():
        return

	def render(self, image):
		defined_colors = {
			'red' : ['rgb', [1, 0, 0]],
			'green' : ['rgb', [0, 1, 0]],
			'blue' : ['rgb', [0, 0, 1]],
			'yellow' : ['rgb', [1, 1, 0]],
			'magenta' : ['rgb', [1, 0 ,1]],
			'cyan' : ['rgb', [0, 1, 1]],
			'white' : ['rgb', [1, 1, 1]],
			'black' : ['rgb', [0, 0, 0]]}
		# draw projected segments
		for segment in self.map_data["segments"]:
            image = self.draw_segment(image, segment)
        return image

    def draw_segment(self, image, segment):
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
		color_type, [r,g,b] = defined_colors[color]
		cv2.line(image, (pt_x[0], pt_y[0]),(pt_x[1], pt_y[1]),(b * 255, g* 255, r * 255), 5)
	return image
