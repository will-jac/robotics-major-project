#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Vector3, Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

import math
import tf.transformations
import numpy as np

import tf2_ros

# from the python3 docs
def isclose(a,b,rel_tol=1e-08, abs_tol=0.01):
    abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

class Navigation():

    # this needs to be big to prevent angle differences from causing problems (-> excessive turning)
    resolution = 0.02

    ang_resolution = 0.1
    # this should be an odd number
    fuzzy_n_divisions = 21

    min_dist_to_nav_to = 0.5 # meters
    min_dist_to_avoid = 0.1

    def __init__(self):
        # print('starting up')

        self.goal = None
        self.target = None
        self.pos = Point()
        self.heading_to_target = 0.0
        self.scan = None
        self.heading = Twist()
        self.odom_counter = 0

        # create a navigation node
        rospy.init_node('navigation', anonymous=True)

        rospy.loginfo('startup navigation')

        rospy.on_shutdown(self.shutdown)

        # listen to the path planner
        rospy.Subscriber('/project/path_planner', Point, self.set_goal)
        rospy.Subscriber('/project/pose', Point, self.update_pos)

        rospy.Subscriber('/scan', LaserScan, self.update_scan)

        # listen to SLAM
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # create some publishers
        self.nav_blocked_publish = rospy.Publisher('/project/replan', Empty, queue_size=1)
        self.vel_publish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.target_request_publish = rospy.Publisher('/project/next_point_in_path', Empty, queue_size=1)

        # print('starting up')

        rate = rospy.Rate(100)
        # Keep self from shutting down until killed
        while not rospy.is_shutdown():
            self.navigate()
            self.vel_publish.publish(self.heading)
            rate.sleep()

    def shutdown(self):
        print('bye bye')

    def navigate(self):
        if self.target is None:
            return
        # if self.is_target_obstructed():
        #     if not self.compute_new_target():
        #         # TODO: tell the path planner no path found -> trigger a replan
        #         self.nav_blocked_publish.publish()
        self.heading_to_target = self.calc_target_angle()

        if self.reached_target():
            # stop moving
            # this is causing problems for me (Jack)
            # if self.target.z == -1:
            #     self.heading = Twist()
            # else:
            #     self.heading = Twist()
            #     self.heading.linear.x = 0.1
            self.heading = Twist()

            # set new target (unless we're already at the goal)
            if self.target.x == self.goal.x and self.target.y == self.goal.y:
                # print('self.target == self.goal')
                # print(self.target)
                # print(self.goal)
                self.target = None
                
                # Send request for next target
                self.target_request_publish.publish()
            else:
                rospy.loginfo('setting target to goal')
                self.target = self.goal
                
        else:
            self.nav_to_target()

    def is_target_obstructed(self):
        if self.scan is None:
            print('scan is none')
            return False

        n = len(self.scan.ranges)
        middle = n // 2
        if n == 0:
            # bad scan
            print('bad scan')
            return False

        # where should the target be in relation to our heading?
        # can we see in the direction of the target?
        
        # if (self.pos.z - self.scan.angle_min) % 3.14 < heading_to_target and \
        #         (self.pos.z - self.scan.angle_max) % 3.14 > heading_to_target:

        # angle_diff < scan_angle_range
        angle_diff = self.heading_to_target - self.pos.z 
        if abs(angle_diff) < abs(self.scan.angle_max - self.scan.angle_min):
            
            # we can see the target
            # what's the scan in that direction?
            offset = int((angle_diff + self.scan.angle_min/ self.scan.angle_increment))
            
            self.scan_target_index = offset + middle
            index_min = min(0, self.scan_target_index - 5)
            index_max = max(n-1, self.scan_target_index + 5)
            # print('angle diff is:', angle_diff, self.heading_to_target, n, middle, index)

            # scan through the ranges around it too
            if not(np.isnan(self.scan.ranges[index_min : index_max]).all()):
                if self.dist_to_target() >= np.nanmin(self.scan.ranges[index_min : index_max]):
                    # there is an object, and it's closer than the target
                    print('target obstructed')
                    print(self.heading_to_target, self.pos.z, angle_diff, self.scan.angle_increment, offset, middle)
                    return True

        # return False if we can't see the target
        # print('target not seen / not obstructed')
        return False

    def compute_new_target(self):
        if self.scan is None:
            print('scan none')
            return True
        # we need to find the new target
        # closest point (in the theta dimension) to the target where scan is nan

        n = len(self.scan.ranges)

        # find objects around us
        objects = [None]*Navigation.fuzzy_n_divisions

        f = Navigation.fuzzy_n_divisions

        for i in range(Navigation.fuzzy_n_divisions):
            # None if there is an object (eg not all values nan)
            # otherwise closest object in the slice
            all_nan = np.isnan(self.scan.ranges[i*f:(i+1)*f]).all()
            if not all_nan:
                objects[i] = np.nanmin(self.scan.ranges[i*f:(i+1)*f])
            else:
                objects[i] = None
        print(objects)
        # determine where to go
        # we want minimal perterbation from the target heading
        scan_target_n = int(math.floor((self.scan_target_index / n) * Navigation.fuzzy_n_divisions))
        print(self.scan_target_index, n, Navigation.fuzzy_n_divisions, scan_target_n)

        # should be false, eg there is an object
        if objects[scan_target_n] is None:
            # no obstruction on way to target
            print('no obstruction on way to target')
            return True
        else:
            # how far?
            if objects[scan_target_n] > self.dist_to_target():
                # we can make it to the target
                print('we can make it to target')
                return True

        # target obstructed

        i = 0
        target_index = None
        for window in range(1,Navigation.fuzzy_n_divisions):
            # look left and right, then expand the window (initially, both these will look at scan_target_n)
            j = i * window
            # first condition is bounds checking the array
            # second is if either there's no the object or the object is further than the target
            if 0 <= scan_target_n - j and (objects[scan_target_n - j] is None 
                    or objects[scan_target_n - j] > Navigation.min_dist_to_avoid):
                print('target found on left', scan_target_n - j)
                # slot has no obstacles
                target_index = scan_target_n - j
                break
            elif n > scan_target_n + j and (objects[scan_target_n + j] is None 
                    or objects[scan_target_n + j] > Navigation.min_dist_to_avoid):
                print('target found on right', scan_target_n + j)
                target_index = scan_target_n + j
                break

        # window found or nothing
        if target_index is None:
            return False
        else:
            # empty thing found at target_index
            # compute new target
            #### ASSUMING THAT LEFT = +, RIGHT = -
            ang_to_index = self.scan.angle_min + target_index * Navigation.fuzzy_n_divisions * self.scan.angle_increment
            print('angle to new target is:', ang_to_index)

            self.target = Point()
            self.heading_to_target = self.pos.z + ang_to_index
            # compute x, y from heading, dist (ie convert radians -> cartesian)
            # this will give us the offset of the target from the robot -> use for abs location
            self.target.x = self.pos.x + Navigation.min_dist_to_nav_to * math.cos(self.heading_to_target)
            self.target.y = self.pos.y + Navigation.min_dist_to_nav_to * math.sin(self.heading_to_target)

            print('new target is', self.target)

        return True

    def reached_target(self):
        if not (self.target is None):
            if abs(self.pos.x - self.target.x) < Navigation.resolution:
                if abs(self.pos.y - self.target.y) < Navigation.resolution:
                    # we're there!
                    rospy.loginfo('reached target')
                    rospy.loginfo(self.pos)
                    return True
        # rospy.loginfo('not at target')
        return False

    def nav_to_target(self):
        # calculate where to go (nav to the target)
        if not (self.target is None):
            if abs(self.pos.x - self.target.x) < Navigation.resolution:
                if abs(self.pos.y - self.target.y) < Navigation.resolution:
                    # at target
                    return

            # should we change the direction we're heading?
            if self.pos.z > 0:
                delta = self.heading_to_target - self.pos.z 
                if self.heading_to_target < 0:
                    # p = +, h = -
                    if abs(delta) > math.pi:
                        # we're close to pi
                        p = 3.14 - self.pos.z
                        h = 3.14 + self.heading_to_target
                        delta = h - p
                    else:
                        delta *= -1
            else:
                delta = self.heading_to_target - self.pos.z 
                if self.heading_to_target > 0:
                    # p = -, h = +
                    if abs(delta) > math.pi:
                        # we're close to pi
                        p = 3.14 - self.pos.z
                        h = 3.14 + self.heading_to_target
                        delta = p - h

            if abs(delta) < Navigation.ang_resolution:
                # print('forward', self.pos.x, self.pos.y, self.pos.z, self.heading_to_target)
                # move forwards
                self.heading.linear.x = .2
                self.heading.angular.z = 0.0
            else:
                # reset heading
                if delta > 0:
                    self.heading.angular.z = 1
                else:
                    self.heading.angular.z = -1
                self.heading.linear.x = 0.0
                
                # print('turning', self.pos.x, self.pos.y, self.pos.z, self.heading_to_target, delta, self.heading.angular.z,)
            # print('naving to target') #, self.heading, self.heading_to_target, self.pos)

    def calc_target_angle(self):
        delta_x = self.target.x - self.pos.x
        delta_y = self.target.y - self.pos.y

        if isclose(delta_x, 0) or delta_x == 0:
            if delta_y < 0:
                dest_ang = -1 * math.pi / 2
            else:
                dest_ang = math.pi / 2
        elif isclose(delta_y, 0) or delta_y == 0:
            if delta_x < 0:
                dest_ang = math.pi
            else:
                dest_ang = 0
        else:
            dest_ang = math.atan(delta_y / delta_x)

            if delta_x < 0.0:
                # flip the unit circle
                if dest_ang > 0:
                    # stay positive
                    dest_ang = math.pi - dest_ang
                else:
                    dest_ang = -math.pi - dest_ang

        # print('heading to target', dest_ang)
        return dest_ang

    def dist_to_target(self):
        return math.hypot(self.pos.x - self.target.x, self.pos.y - self.target.y)

    def set_goal(self, dest_msg):
        rospy.loginfo('recieved goal:')
        rospy.loginfo(dest_msg)
        rospy.loginfo('pos:')
        rospy.loginfo(self.pos)

        self.goal = dest_msg
        self.target = dest_msg

        self.heading_to_target = self.calc_target_angle()

    def update_scan(self, scan):
        if len(scan.ranges) == 0:
            # bad scan
            return
        self.scan = scan

    def update_pos(self, data):
        self.pos = data

if __name__ == "__main__":
    Navigation()