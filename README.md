
# Robotics Major Project

## Modules

### SLAM

After a lot of trial and error, we're using `rgbdslam_v2` for SLAM. This uses ORB SLAM 2 and OctoMaps on the backend (coolness factor: through the roof).

Note: I'm writing this README at 3:30 AM after finally getting this working, so I don't apologize for the dumb jokes.

[RGBD SLAM v2](https://github.com/felixendres/rgbdslam_v2) is a SLAM library created by turtlebots, for turtlebots. It's crazy optimized, and has a ton of configuration options for running it. It also comes with a very cool visualization UI.

#### Installation

Installing this is complicated. Hang with me, and make sure each step is followed

* install openni `sudo apt-get install ros-kinetic-openni-launch`
* remove EVERYTHING in your catkin workspace that could contain g2o (eg: another slam library you tried to install that didn't work)
* clean the workspace: `rm -r build devel install`
* run the custom install script in this repo (from anywhere)
```{bash}
chmod +x install_rgbslam.sh./install_rgbslam.sh
```
* follow [this](https://github.com/felixendres/rgbdslam_v2/wiki/Instructions-for-Compiling-Rgbdslam-(V2)-on-a-Fresh-Ubuntu-16.04-Install-(Ros-Kinetic)-in-Virtualbox) tutorial on steps 11-16

Yay! RGBDSLAM should be installed!

#### Configuration

Use the dedicated launch file in the launch directory

#### Usage

Okay, so we can see the cool UI. But what the heck does that mean?

Well, RGBDSLAM doesn't actually directly publish the information we really want (a map and pose). What it does give us is a few things:

* A pointcloud that we can use to build into our own map (via a custom mapping node or an octomap server).
* A map that we can transform the pose estimate to (basically).
* A service to save an octomap to a file.

How you use these things is up to you. I'd recommend the following strategy:

* Transform the pose estimate to get an actual pose on demand (eg: before each time you send a velocity command to the robot).
* Run a dedicated octomap server to incrementally build the octomap - this will publish an occupancy grid for us (!!)

To get the pose of the robot, you can add the following code:

```{python}
from geometry_msgs.msg import Point
import tf2_ros
import tf.transformations

class RosNode:

  def __init__():
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    self.pos = Point

    rospy.init_node('some_ros_node', anonymous=True)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
      try:
        trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        # cool, now we have the transform
        # this gives us a translation / rotation from wherever the robot started (0,0,0) to where it is now
        # TODO: account for inital position (can use /initial_position topic)

        self.pos.x = trans.transform.translation.x
        self.pos.y = trans.transform.translation.y
        # convert to euler - could also do everything in quaternion
        r = trans.transform.rotation
        euler = tf.transformations.euler_from_quaternion([r.x, r.y, r.z, r.w])
        self.pos.z = euler[2]
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue
      
      # do whatever you need to with the robot's position

      rate.sleep()
```

### Octomap

In order to get an occupancy grid out of the rgbdslam, we need to run an octomap server to take in the point cloud and build a map from it.

#### Installation

```{bash}
sudo apt install ros-kinetic-octomap-server
```

#### Configuration

Add this to your launch file:

```{xml}
  <!-- Octomap -->
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    <param name="resolution" value="0.05" />
    <!-- Fixed frame ID -->
    <param name="frame_id" type="string" value="map" />
    <!-- Max range to integrate -->
    <param name="sensor_model/max_range" value="5.0" />
    <!-- remap to the rgbdslam PointCloud2 topic -->
    <remap from="cloud_in" to="rgbdslam/online_clouds"/>
  </node>
```

#### Usage

Now, you can get the occupancy grid from the topic `/projected_map` - it's published as a [`/nav_msgs/OccupancyGrid`](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html) message.

## Running this project

```{bash}
roslaunch robotics-major-project project.launch
```
## Issues

None yet

export ROBOT_INITIAL_POSE="-x 2 -y 3 -Y 0"


## Other SLAM options explored

### Gmapping

Gmapping is super integrated with ROS. That's great!

However, it sucks for turtlebots (it's bad at navigating). That sucks!

#### Configuration

To add `gmapping` to your project, add the following to your launch file:

```{xml}
<node pkg="gmapping" type="slam_gmapping" name="slam_gmapping">
  <param name="scan" value="base_scan"/>
</node>
```

If (like me) you want to pull up `Rviz` to visualize mapping in real time, add this:

```{xml}
<launch>
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find turtlebot_rviz_launchers)/rviz/navigation.rviz"/>
</launch>
```

or just run:

```{bash}
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

in a new terminal.

#### Usage

The two important things we want out of `gmapping`, are the robot's pose estimate and the occupancy grid it creates.

For the occupancy grid, `gmapping` publishes a `nav_msgs/OccupancyGrid` message to the `/map` topic. 

What about the robot's pose? Well... gmapping doesn't really do that. You can transform the `/odom` frame into the `/map` to get it, but I can't really get that to work.

In comes: `acml`. 

### acml

[acml](http://wiki.ros.org/amcl) is used to actually ge the robot's position, using the map and transformation provided by `gmapping`.

#### Configuration

To add `acml` to your project, add the following to your launch file:


### Hector SLAM

[hector slam](http://wiki.ros.org/hector_slam) is a SLAM library that is hyper-optimized for pan- and tilt-locked laser scanners (eg: laser scanners that only move horizontally). This is great for us, because that's exactly what we're using! 

However, `hector_slam` is designed to work for long-range, wide-angle, low-noise, LIDAR-style laser scanners. The kinetic is the *exact* opposite of this.

Also, I just *could not* get this to work. See the `launch/hector.launch` file if you want to try it yourself.

To install the package, just run:

```{bash}
sudo apt-get install ros-kinetic-hector-slam
```

I faced some issue with installing it (my computer was having issues connecting to ros.packages.org, because of something weird with IPv6): if this is happening to you, you can install [from source](https://github.com/tu-darmstadt-ros-pkg/hector_slam). Just `git clone` into `~/catkin_ws/src/` then run `catkin_make -DCMAKE_BUILD_TYPE=Release` from `~/catkin_ws`.

#### Test Hector SLAM

Following [this] tutorial:

1. Download a bagfile of messages so we can rule out any error involved with the robot

```{bash}
wget https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/tu-darmstadt-ros-pkg/Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag
```

2. Start `hector_slam`

```{bash}
roslaunch hector_slam_launch tutorial.launch
```

3. Visualize the mapping in Rviz. From another terminal:

```{bash}
rosbag play Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag  --clock
```

#### Configuration

`hector_slam` has many, many options for running. It's a meta package, which means that it contains a lot of other packages inside of it that we can run. You can read more about them [here](http://wiki.ros.org/hector_mapping).

Some notable options:

* `hector_mapping` is SLAM without odometry. It requires low resources, subscribes to the laser scanner (`/scan`), and outputs a map (`/map`, [nav_msgs/OccupancyGrid](docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html)) and estimated pose (`/slam_out_pose`, [geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)), among other things.

#### Launch file

To run the hector slam in simulation, we need to make a few modifications to avoid some problems with the time. Run:

```{bash}
rosedit 
```
  <arg name="odom_frame" default="odom"/>

Add this to your launch file:

```{xml}
  <include file="$(find hector_slam_launch)/launch/tutorial.launch"/>
```

To launch `hector_slam` and Rviz, so you can see the output. If you don't like Rviz launching, replace the line with this:

```{xml}
  <arg name="geotiff_map_file_path" default="$(find hector_geotiff)/maps"/>

  <param name="/use_sim_time" value="true"/>

  <include file="$(find hector_mapping)/launch/mapping_default.launch"/>

  <include file="$(find hector_geotiff)/launch/geotiff_mapper.launch">
    <arg name="trajectory_source_frame_name" value="scanmatcher_frame"/>
    <arg name="map_file_path" value="$(arg geotiff_map_file_path)"/>
  </include>
```

### ORB_SLAM_2

UPDATE: We are no longer using ORB SLAM 2, due to issues with mono crashing that we were unable to fix. We were able to get rgbd running, but it never was able to actually finish localizing robot.

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

#### Configuration and Launch

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

##### Camera

The mono and RGBD cameras are both working now - see the launch directory

##### Odometer

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

##### Occupancy grid

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

