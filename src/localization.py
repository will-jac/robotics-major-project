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
        position = odom.pose.pose.position
        self.pos.x = position.x + self.offset.x
        self.pos.y = position.y + self.offset.y

        r = odom.pose.pose.orientation
        self.pos.z = tf.transformations.euler_from_quaternion((r.x, r.y, r.z, r.w))[2]
        # print(self.pos.z)
        self.pos.z += self.offset.z

        self.position_publisher.publish(self.pos)

    def tf_callback(self, msg):
        for tfm in msg.transforms:
            if tfm.child_frame_id == 'odom' and tfm.header.frame_id == 'map':
                # print(tfm.transform.translation)
                self.offset = tfm.transform.translation
                break

    def __init__(self):
        self.pos = Point()
        self.offset = Point()

        rospy.init_node('localization_node', anonymous=True)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)

        self.position_publisher = rospy.Publisher("/project/pose", Point, queue_size=1)

        rospy.spin()


if __name__ == '__main__':
    Localization()