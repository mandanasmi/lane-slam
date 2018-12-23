#!/usr/bin/env python
import math
import numpy as np
import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def rotate_point(px, py, cx, cy, theta):
    """
    Rotate a 2D point around a center
    """

    dx = px - cx
    dy = py - cy

    new_dx = dx * math.cos(theta) + dy * math.sin(theta)
    new_dy = dy * math.cos(theta) - dx * math.sin(theta)

    return cx + new_dx, cy + new_dy

def get_dir_vec(angle):
    """
    Vector pointing in the direction the agent is looking
    """

    x = math.cos(angle)
    y = -math.sin(angle)
    return np.array([x, y])

def get_right_vec(angle):
    """
    Vector pointing to the right of the agent
    """

    x = math.sin(angle)
    y = math.cos(angle)
    return np.array([x, y])


class OdometryNode(object):
    def __init__(self):
        self.sub_lanefilter=rospy.Subscriber('~wheels_cmd',WheelsCmdStamped, self.getPose, queue_size=1)
        self.pub_traj = rospy.Publisher("~trajectory", Marker, queue_size=1)

        self.pos = [0.0,0.0]
        self.theta = 0.0
        self.last_t=rospy.Time.now().nsecs/1e9
        self.dt = 0.1

        self.traj_mark = Marker()
        self.traj_mark.header.stamp=rospy.Time.now()
        self.traj_mark.id = 0
        self.traj_mark.header.frame_id = '/map'
        self.traj_mark.ns="odom_trajectory"
        self.traj_mark.scale.x = 0.1
        self.traj_mark.color.a = 1.0
        self.traj_mark.color.b = 1.0
        self.traj_mark.type=self.traj_mark.LINE_STRIP
        self.traj_mark.action=self.traj_mark.ADD


    def drive(self,Vl,Vr):
        """
        Drive this bad boy
        """

        cur_pos = np.array(self.pos)
        cur_angle = self.theta
        l = 0.5

        # If the wheel velocities are the same, then there is no rotation
        if Vl == Vr:
          cur_pos = cur_pos + self.dt * Vl * get_dir_vec(cur_angle)
          return cur_pos, cur_angle

        # Compute the angular rotation velocity about the ICC (center of curvature)
        w = (Vr - Vl) / l


        # Compute the distance to the center of curvature
        r = (l * (Vl + Vr)) / (2 * (Vl - Vr))

        # Compute the rotation angle for this time step
        rotAngle = w * self.dt

        # Rotate the robot's position around the center of rotation
        r_vec = get_right_vec(cur_angle)
        px, py = cur_pos
        cx = px + r * r_vec[0]
        cy = py + r * r_vec[1]
        npx, npy = rotate_point(px, py, cx, cy, rotAngle)
        cur_pos = np.array([npx, npy])

        # Update the robot's direction angle
        cur_angle += rotAngle
        return cur_pos, cur_angle

    def showTraj(self):
        position=Point()
        position.x = self.pos[0]
        position.y = self.pos[1]
        position.z = 0.0
        self.traj_mark.points.append(position)
        self.pub_traj.publish(self.traj_mark)

    def getPose(self,wheels_cmd):
        self.dt=wheels_cmd.header.stamp.nsecs/1e9 - self.last_t
        self.last_t=wheels_cmd.header.stamp.nsecs/1e9
        if (self.dt > 0.0) and (self.dt < 0.3):
            self.pos,self.theta = self.drive(wheels_cmd.vel_left,wheels_cmd.vel_right)
            br.sendTransform((self.pos[0], self.pos[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.theta),
                         rospy.Time.now(),
                         "duck",
                         "map")
            self.showTraj()


if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    br = tf.TransformBroadcaster()
    odometry_node = OdometryNode()
    rospy.spin()

