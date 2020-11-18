
#

## Installation

### Install ORB_SLAM_2

follow the instructions at:

https://github.com/appliedAI-Initiative/orb_slam_2_ros

(note: the instructions say to run `catkin build`, but the appropriate command is `catkin_make_isolated`)

#### Test orb slam

In seperate terminals, run:

```{bash}
roscore
```

```{bash}
roslaunch turtlebot_gazebo turtlebot_world.launch
```

```{bash}
roslaunch orb_slam2_ros orb_slam2_r200_mono.launch
```

```{bash}
roslaunch turtlebot_teleop keyboard_teleop.launch
```

And move the robot around until the message `Map point vector is empty!` is empty (the message just means the map isn't big enough for SLAM yet).

We frequently ran into [this](https://github.com/appliedAI-Initiative/orb_slam_2_ros/issues/88) error. In this case, try adding `--screen` to the end of the `orb_slam_2` launch command, and recompile with `catkin_make_isolated`. You can also test [these](https://www.hackadda.com/post/2020/5/17/using-orb-slam-ros-to-map-the-world/) instructions for turtlebot3

## Configuration

### ORB SLAM 2

Add the following lines to your launch file:

```{bash}
  <include file="$(find orb_slam2_ros)/ros/launch/orb_slam2_r200_mono.launch">
    <arg name="load_map" value="true" />
    <arg name="map_file" value="major_project_map" />
  </include>
```

## Usage

### ORB SLAM 2

First, you'll need to drive around enough to create a full enough map for localization. After this, you shouldn't need to again, because ORB SLAM2 will save the map file for you on exit.

### Odometer

ORB SLAM2 publishes its odometer to `/orb_slam2_mono/pose`, using the `Odometer` message. To read this, you can use the following code snippet to store the `Pose` component of the message.

```{python}
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf.transformations

class Navigation():
    def __init__(self):
        self.pose = None
        rospy.init_node('navigation', anonymous=False)
        rospy.Subscriber('/orb_slam2_mono/pose', Odometry, self.update_pos)

    def update_pos(self, odom):
        self.pose = odom.pose.pose
```

Note that a `Pose` message contains the heading in quaternion form, which is useful when you're in 3D space, but is kind of difficult to work with. If, like me, you prefer in 'Euler' heading (eg: heading as a single theta value), you can use this transformation code to store the heading as the `z` component of the `Position` (making it an `x`,`y`,`z` struct).

```{python}
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf.transformations

class Navigation():
    def __init__(self):
        self.position = None
        rospy.init_node('navigation', anonymous=False)
        rospy.Subscriber('/orb_slam2_mono/pose', Odometry, self.update_pos)

    def update_pos(self, odom):
        self.position = odom.pose.pose
        o = odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.position.z = euler[2] + self.init.z
```

One thing to note is that the odometer only updates if ORB SLAM knows where the robot is! If localization fails, then the odometer will not be updated. We can use the robot's odometer as a fallback here. Currently testing a few different options for how to best do this (although it doesn't seem to occur frequently) - check back soon for an update.

### Occupancy grid

We're working on a way for ORB SLAM to output its updated map in realtime. This functionality is provided in a [different](https://github.com/rayvburn/ORB-SLAM2_ROS) ORB SLAM repo, but the installation and configuration process for that repo is much more complicated (and even undocumented).

Right now, the best way to update D* with new data about where objects are located is directly using the sensors on the robot. Check back soon for more updates about this configuration.

