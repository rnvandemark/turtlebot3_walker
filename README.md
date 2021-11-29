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

### Sourcing Your Environment

```
# It's recommended you do this with every shell opened
source /opt/ros/melodic/setup.bash
```

### Building the Program

```
# Navigate to a catkin workspace's (create if necessary) src directory
cd /path/to/catkin_ws/src
git clone https://github.com/rnvandemark/turtlebot3_walker.git
cd ..
catkin_make install
```

### Running a Sample of the Program

- Set the following environment variable as such:
```
export TURTLEBOT3_MODEL=burger
```
- Source the local workspace (as well as the ROS installation):
```
source /path/to/catkin_ws/install/setup.bash
```
- A launch file is available to run the walker algorithm with a visualization:
```
roslaunch turtlebot3_walker main.launch
```
- Launch file arguments:
  - record (value of either 0 or 1, 0 by default): If set to 1, a rosbag will
    be recorded
  - do_rqt_console (value of either 0 or 1, 0 by default): If set to 1, a rqt
    node/windows will be started as well
  - screen_output (value of either 0 or 1, 1 by default): If set to 1, the
    walker node will log to the screen, otherwise will log to a log file

### Using ROS Bags

Run the simulation and record a ROS bag of your own with the following command:
```
roslaunch turtlebot3_walker main.launch record:=1
```

The bag file should be created in the ROS environment root folder. A sample bag
file can also be seen in the results directory of this package.

To dump information on a desired bag file, run the following command:
```
rosbag info /path/to/file.bag
```

And to play back the recorded sequence of events, run the following command:
```
# WARNING: no Gazebo simulator should be running when entering this command!
rosbag play /path/to/file.bag
```
