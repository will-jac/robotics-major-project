
#

## Instalation

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
