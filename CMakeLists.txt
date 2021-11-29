cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_walker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
)
find_package(Boost)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(turtlebot3_walker src/WalkerNode.cpp src/WalkerController.cpp)
target_link_libraries(turtlebot3_walker ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
    DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    TARGETS turtlebot3_walker
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
