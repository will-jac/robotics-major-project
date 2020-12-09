#!/usr/bin/env python

import rospy
from math import degrees
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class Localization():

    def odom_callback(self, odom):
        orientation = odom.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        self.pos.z = tf.transformations.euler_from_quaternion(quaternion)[2]

        position = odom.pose.pose.position
        self.pos.x = position.x + offset[0]
        self.pos.y = position.y + offset[1]
        pos_pub.publish(self.pos)

    def tf_callback(self, msg):
        for tfm in msg.transforms:
            if tfm.child_frame_id == 'odom' and tfm.header.frame_id == 'map':
                self.offset = (tfm.transform.translation.x, tfm.transform.translation.y)
                break

    def __init__():
        self.pos = Point()
        self.offset = (0,0)
        
        # initialize node
        rospy.init_node('localization_node', anonymous=True)

        # subscribe to odom
        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/tf", TFMessage, tf_callback)

        # publish current heading and position
        position_publisher = rospy.Publisher("/project/point", Point, queue_size=1)

        # cycle through callbacks
        rospy.spin()


if __name__ == '__main__':
    try:
        Localization()
    except rospy.ROSInterruptException:
        pass