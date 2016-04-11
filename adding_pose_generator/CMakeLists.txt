cmake_minimum_required(VERSION 2.8.3)
project(adding_pose_generator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  std_msgs
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    std_msgs
    gazebo_plugins
  DEPENDS
    gazebo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(adding_pose_generator_node src/adding_pose_generator_node.cpp)
target_link_libraries(adding_pose_generator_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS adding_pose_generator_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
