#!/usr/bin/env python
import unittest, rosunit
from anti_instagram import (L1_image_distance, L2_image_distance, logger,
    random_image, scaleandshift1, scaleandshift2, wrap_test_main)
import numpy as np

class AntiInstagramCorrectnessTest(unittest.TestCase):

    def assert_L1_small(self, img1, img2, threshold=0.1):
        diff_L1 = L1_image_distance(img1, img2)
        diff_L2 = L2_image_distance(img1, img2)
        logger.info('diff_L2: %f' % diff_L2)
        logger.info('diff_L1: %f' % diff_L1)
        self.assertLessEqual(diff_L1, threshold)

    def test_anti_instagram_correctness(self):
        logger.info('This is going to test that algorithm 1 and 2 give same results')

        id_scale, id_shift = [1.0, 1.0, 1.0], [0.0, 0.0, 0.0]
        img = random_image(480, 640)

        logger.info('algo 1 respects the identity')
        a = scaleandshift1(img, id_scale, id_shift)
        self.assert_L1_small(img, a)

        logger.info('algo 2 respects the identity')
        b = scaleandshift2(img, id_scale, id_shift)
        self.assert_L1_small(img, b)

        logger.info('algo 1 and 2 give the same output with random shift')

        scale = id_scale
        shift = np.random.rand(3)

        img1 = scaleandshift1(img, scale, shift)
        img2 = scaleandshift2(img, scale, shift)
        self.assert_L1_small(img1, img2)


        logger.info('algo 1 and 2 give the same output with random scale')

        scale = np.random.rand(3)
        shift = id_shift  # 0 shift

        img1 = scaleandshift1(img, scale, shift)
        img2 = scaleandshift2(img, scale, shift)
        self.assert_L1_small(img1, img2)



        logger.info('algo 1 and 2 give the same output with random inputs')

        scale = np.random.rand(3)
        shift = np.random.rand(3)

        img1 = scaleandshift1(img, scale, shift)
        img2 = scaleandshift2(img, scale, shift)
        self.assert_L1_small(img1, img2)

if __name__ == '__main__':
    rosunit.unitrun('anti_instagram', 'antiinstagram_correctness_test', AntiInstagramCorrectnessTest)