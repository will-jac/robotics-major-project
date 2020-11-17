#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import tf.transformations
import numpy as np

class Navigation():

    resolution = 0.01
    fuzzy_n_divisions = 5

    def __init__(self):
        self.dest = None
        self.pos = None
        self.head = Twist()
        # create a navigation node
        rospy.init_node('navigation', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # listen to the path planner
        rospy.Subscriber('/project/path_planner', Vector3, self.set_dest)
        rospy.Subscriber('/odom', Odometry, self.update_pos)

    def set_dest(self, dest_msg):
        rospy.loginfo('recieved dest:')
        rospy.loginfo(dest_msg)
        self.dest = dest_msg

    def update_pos(self, odom):
        self.pos = odom.pose.pose.position
        
        # could also do all the calculations in quaternion form
        o = odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.pos.z = euler[2] + self.init.z

        # calculate where to go
        if not (self.dest is None):
            
            if abs(self.pos.x - self.dest.x) < Navigation.resolution:
                if abs(self.pos.y - self.dest.y) > Navigation.resolution:
                    # we're there!
                    rospy.loginfo('reached dest')
                    rospy.loginfo(self.pos)

                    #TODO: Call the execution monitor
                    return

            # we're not there
            
            # should we change the direction we're heading?
            heading_to_target = calc_heading()
            if abs(self.pos.z - heading_to_target) % 3.14 < Navigation.resolution:
                # reset heading
                self.head.z = heading_to_target
            
    def update_scan(self, scan):
        self.scan = scan

        # need to run AVOID here

        n = len(scan.ranges) / Navigation.fuzzy_n_divisions

        if n == 0:
            # bad scan
            return

        if abs(self.pos.z - self.head.z) % 3.14 < Navigation.resolution:
            # we're turning, so we don't actually care about the scan
            

        # find objects around us
        objects = [None]*n
        for i in range(Navigation.fuzzy_n_divisions):
            # true if there is an object (eg not all values nan)
            objects[i] = not(np.isnan(scan.ranges[i*n:(i+1)*n]).all())

        # determine where to go
        if objects[n//2]:
            # something in front of us - can we move around it?


    def calc_heading():
        delta_x = self.dest.x - self.pos.x
        delta_y = self.dest.y - self.pos.y

        if delta_x == 0:
            if delta_y < 0:
                dest_ang = math.pi / 2
            else:
                dest_ang = -1 * math.pi / 2
        elif delta_y == 0:
            if delta_x < 0:
                dest_ang = math.pi
            else:
                dest_ang = 0
        else:
            dest_ang = math.atan(delta_y / delta_x)

            if delta_x < -0.0001:
                rospy.loginfo('delta_x = ' + str(delta_x))
                # flip the unit circle
                if dest_ang > 0:
                    # become negative
                    dest_ang -= math.pi
                else:
                    # become positive
                    dest_ang += math.pi

        return dest_ang