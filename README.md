
#

## Installation

### Install ORB_SLAM_2

Follow the instructions at:

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

#### Launch

For a basic launch file + test, add the following line to your launch file:

```{xml}
  <include file="$(find orb_slam2_ros)/ros/launch/orb_slam2_r200_mono.launch"/>
```

However, the launch file they provide is very basic, and has no argument support. We want to be able to load in a map to use, so pull in the launch file from the launch folder in this repo. It sets the parameter `load_map` to true, which will try to load a `map.bin` file from your home directory (modify if you want to save it elsewhere).

Finally, we need to change our launch file so that it calls this one. Edit the line to:

```{xml}
  <include file="orb_slam2_turtlebot_mono.launch"/>
```

Now, when we run our launch file, it will launch ORB SLAM, which will load a map called `map.bin` from your home directory, if such a map exists.

Note that you should probably configure the project launch file so it dynamically finds the turtlebot launch.

#### Camera

As of right now, the turtlebot will only work with the mono camera. However, the turtlebots do have an RGBD camera, so this should work with ORB SLAM. We're working on the configuration right now--there seems to be some (undocumented) issue with the launch file.

Additionally, we will get better results if we get the calibration information from our camera. These are set for you in the turtlebot launch file. If you want to check, they come from the `/camera/rgb/camera_info` topic

## Usage

### ORB SLAM 2

First, you'll need to drive around enough to create a full enough map for localization. After this, you shouldn't need to again, because ORB SLAM2 will save the map file for you on exit.

### Odometer

ORB SLAM2 publishes its odometer to `/orb_slam2_mono/pose`, using the [Odometer](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) message. To read this, you can use the following code snippet to store the [Pose](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html) component of the message.

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

Note that a `Pose` message contains the heading in quaternion form, which is useful when you're in 3D space, but is kind of difficult to work with. If, like me, you prefer in 'Euler' heading (eg: heading as a single theta value), you can use this transformation code to store the heading as the `z` component of the [Point](http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html) (making it an `x`,`y`,`z` struct).

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

ORB SLAM2 uses an occupancy grid, which is fine for 2D maps.

We're working on a way for ORB SLAM to output its updated map in realtime. This functionality is provided in a [different](https://github.com/rayvburn/ORB-SLAM2_ROS) ORB SLAM repo, but the installation and configuration process for that repo is much more complicated (and even undocumented).

Right now, the best way to update the path planner (or whatever node needs this information) with new data about where objects are located is to use the save map [service](http://wiki.ros.org/Services) to save the map to the file, then read in the new file in the node.

While the map is saving, SLAM is innactive, so the robot shouldn't be moving while this happens.

```{python}
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from orb_slam2_ros.srv import SaveMap

class PathPlanner:
    def __init__():
        self.position = None
        rospy.init_node('path_planner', anonymous=False)
        rospy.Subscriber('/path_planner', Point, self.plan_path)
        
        # create a service proxy for saving the map
        rospy.wait_for_service('/orb_slam2_mono/save_map')
        map_service = rospy.ServiceProxy('/orb_slam2_mono/save_map', SaveMap)

        rospy.spin()

    def plan_path(self, point):
        self.dest = point

        # get the updated map
        try:
            resp = map_service('~/catkin_ws/src/project_name/map.bin')
        except rospy.ServiceException as exc:
            print('Service did not process request: ' + str(exc))
        
        if not resp:
            print('map saving failed')
        
        # TODO: do path planning on the map
```

This should block until the map has finished saving.

## Issues

ORB SLAM sometimes dies, seeming randomly. I'm not sure why this happens, although I suspect it is due to a bad camera configuration / event.
