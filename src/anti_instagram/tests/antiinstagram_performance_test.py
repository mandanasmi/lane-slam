#!/usr/bin/env python
import unittest, rosunit
from anti_instagram import wrap_test_main, SASParams, logger, random_image, AntiInstagram
import timeit

class Params:
    shape = None

def setup():
    img = random_image(Params.shape[0], Params.shape[1])
    ai = AntiInstagram()
    return ai, img

class AntiInstagramPerformanceTest(unittest.TestCase):
    def test_anti_instagram_performance(self):
        logger.info('This is going to test the performance of algorithm 1 and 2')

        for i in [1,2]:
            SASParams.algorithm = i
            shapes = [ (480, 640), (240, 320), (120, 160) ]

            for Params.shape in shapes:
                res = self.applyTransformOnRandomImg()
                #logger.info('algo: %d Shape: %s   -> %1.f ms' % (i, str(Params.shape), 1000*res))
                # self.assertLess(res, 0.05)  # Calculate in less than 0.05

        for Params.shape in shapes:
            res = self.calcuateTransformOnRandomImg()
            #logger.info('Shape: %s   -> %1.f ms' % (str(Params.shape), 1000*res))
            # self.assertLess(res, 0.5)   # Calculate in less than 0.5 seconds

    def applyTransformOnRandomImg(self):
        n = 50
        tn = timeit.timeit(stmt='ai.applyTransform(img)',
                           setup='from __main__ import setup; ai,img=setup()',
                           number=n
                           )
        t = tn / n
        #logger.info("Average Apply Transform Took: %.1f ms " % (t * 1000))
        return t

    def calcuateTransformOnRandomImg(self):
        n = 10
        tn = timeit.timeit(stmt='ai.calculateTransform(img,True)',
                           setup='from __main__ import setup; ai,img=setup()',
                           number=n)

        t = tn / n
        #logger.info("Average Calculate Transform Took: %.1f ms" % (t * 1000))
        return t

if __name__ == '__main__':
    rosunit.unitrun('anti_instagram', 'aniinstagram_performance_test', AntiInstagramPerformanceTest)