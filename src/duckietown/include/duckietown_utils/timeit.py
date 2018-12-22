from contextlib import contextmanager
import time


@contextmanager
def rospy_timeit_clock(s):
    import rospy
    t0 = time.clock()
    yield
    delta = time.clock() - t0    
    rospy.loginfo('%10d ms: %s' % ( 1000*delta, s))

@contextmanager
def rospy_timeit_wall(s):
    import rospy
    t0 = time.time()
    yield
    delta = time.time() - t0    
    rospy.loginfo('%10d ms: %s' % ( 1000*delta, s))