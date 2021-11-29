# turtlebot3_walker

## Project Overview/Description

This is a simple example of working with Gazebo. A 'path' planning algorithm is
implemented, and the various ROS connections are made so input data can come
from the simulated environment and Turtlebot3 Burger, while velocity commands
can go back to the simulated robot.

## Dependencies

- Ubuntu 18.04
- ROS Melodic (Desktop)
- Turtlebot3 and Gazebo simulation packages:
```
sudo apt-get install ros-melodic-turtlebot3* ros-melodic-gazebo*
```

## How to

### Building the Program

```
# Navigate to a catkin workspace's (create if necessary) src directory
cd /path/to/catkin_ws/src
git clone https://github.com/rnvandemark/turtlebot3_walker.git
cd ..
catkin_make install
```

### Running a Sample of the Program

TODO
