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
        self.goal = None
        self.target = None
        self.pos = None
        self.heading = Twist()
        self.odom_counter = 0

        
        # create a navigation node
        rospy.init_node('navigation', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # listen to the path planner
        rospy.Subscriber('/project/path_planner', Vector3, self.set_goal)

        rospy.Subscriber('/odom', Odometry, self.update_odom_pos)
        rospy.Subscriber('/orb_slam2_mono/pose', Odometry, self.update_pos)

        # create some publishers
        self.nav_blocked_publish = rospy.Publisher('/navigation/blocked', Empty)
        self.vel_publish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        rate = rospy.Rate(10)
        # Keep self from shutting down until killed
        while not rospy.is_shutdown():
            navigate()
            self.vel_publish.publish(self.heading)
            rate.sleep()

    def navigate(self):
        if is_target_obstructed():
            compute_new_target()
        if reached_target():
            # set new target (unless we're already at the goal)
            self.target = self.goal
        else:
            nav_to_target()

    def is_target_obstructed(self):
        if len(self.scan.ranges) == 0:
            # bad scan
            return False

        # where should the target be in relation to our heading?
        # can we see in the direction of the target?
        heading_to_target = self.calc_heading()
        # if (self.pos.z - self.scan.angle_min) % 3.14 < heading_to_target and \
        #         (self.pos.z - self.scan.angle_max) % 3.14 > heading_to_target:
        
        # angle_diff < scan_angle_range
        angle_diff = self.pos.z - heading_to_target
        if angle_diff < self.scan.angle_max - self.scan.angle_min
            # we can see the target
            # what's the scan in that direction?
            index = angle_diff // self.scan.angle_increment
            if index > 5 and index < self.scan.range_max - 5:
                # scan through the ranges around it too 
                if not(np.isnan(self.scan.ranges[index-5:index+5]).all()):
                    # there is an object!
                    return True

        # return False if we can't see the target 
        return False
                    
    def reached_target(self):
        if not (self.target is None):
            if abs(self.pos.x - self.target.x) < Navigation.resolution:
                if abs(self.pos.y - self.target.y) > Navigation.resolution:
                    # we're there!
                    rospy.loginfo('reached target')
                    rospy.loginfo(self.pos)
                    return True
        return False

    def nav_to_target(self):
        # calculate where to go (nav to the target)
        if not (self.target is None):
            if abs(self.pos.x - self.target.x) < Navigation.resolution:
                if abs(self.pos.y - self.target.y) > Navigation.resolution:
                    # at target
                    return
            
            # should we change the direction we're heading?
            heading_to_target = calc_heading()
            if abs(self.pos.z - heading_to_target) % 3.14 < Navigation.resolution:
                # reset heading
                self.heading.angular.z = heading_to_target
                self.heading.linear.x = 0.0
            else:
                # move forwards
                self.heading.linear.x = 0.5
                self.heading.angular.z = 0.0
    
    def calc_heading(self):
        delta_x = self.target.x - self.pos.x
        delta_y = self.target.y - self.pos.y

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

    def set_goal(self, dest_msg):
        rospy.loginfo('recieved goal:')
        rospy.loginfo(dest_msg)
        self.goal = dest_msg
        self.target = self.goal

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
           
    def update_scan(self, scan):
        if len(self.scan.ranges) == 0:
            # bad scan
            return 
        self.scan = scan

    