cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_walker)

find_package(catkin REQUIRED
    roscpp
)


catkin_package(
#    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

include_directories(
#    include
    ${catkin_INCLUDE_DIRS}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
