#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

import math
import tf.transformations
import numpy as np

class Navigation():

    resolution = 0.01
    fuzzy_n_divisions = 5
    odom_timeout = 10

    def __init__(self):
        self.dest = None
        self.pos = None
        self.head = Twist()
        self.odom_counter = 0

        
        # create a navigation node
        rospy.init_node('navigation', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # listen to the path planner
        rospy.Subscriber('/project/path_planner', Vector3, self.set_dest)

        rospy.Subscriber('/odom', Odometry, self.update_odom_pos)
        rospy.Subscriber('/orb_slam2_mono/pose', Odometry, self.update_pos)

        # create some publishers
        self.nav_blocked_publish = rospy.Publisher('/navigation/blocked', Empty)
        self.vel_publish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        rate = rospy.Rate(10)
        # Keep self from shutting down until killed
        while not rospy.is_shutdown():
            self.vel_publish.publish(self.head)
            rate.sleep()


    def set_dest(self, dest_msg):
        rospy.loginfo('recieved dest:')
        rospy.loginfo(dest_msg)
        self.dest = dest_msg

    def update_odom_pos(self, odom):
        self.odom_counter += 1
        # have we heard from orbslam recently?
        if self.odom_counter > Navigation.odom_timeout:
            self.update_pos(odom)


    def update_pos(self, odom):
        self.pos = odom.pose.pose.position
        
        # we just heard from orbslam
        self.odom_counter = 0

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
                self.head.angular.z = heading_to_target
                self.head.linear.x = 0.0
            else:
                # move forwards
                self.head.linear.x = 0.5
                self.head.angular.z = 0.0
            
    # obstacle avoidance
    def update_scan(self, scan):
        self.scan = scan

        n = len(scan.ranges) / Navigation.fuzzy_n_divisions

        if n == 0:
            # bad scan
            return

        if abs(self.pos.z - self.head.angular.z) % 3.14 < Navigation.resolution:
            # we're turning, so we don't actually care about the scan
            return

        # find objects around us
        objects = [None]*n
        for i in range(Navigation.fuzzy_n_divisions):
            # true if there is an object (eg not all values nan)
            objects[i] = not(np.isnan(scan.ranges[i*n:(i+1)*n]).all())

        # determine where to go
        if objects[n//2]:
            # something in front of us - report back to path planner
            # TODO

            # currently no classification of people - let's just say that's out of scope
            # (no dynamic / moving objects in our environment)
            self.nav_blocked_publish.publish()

            # need to run AVOID here
            # stop moving
            self.head.linear.x = 0

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