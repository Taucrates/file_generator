cmake_minimum_required(VERSION 3.0.2)
project(file_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  tf2
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES file_generator
  CATKIN_DEPENDS roscpp rospy std_msgs tf2_ros tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pose_file_generator src/pose_file_generator.cpp)
target_link_libraries(pose_file_generator ${catkin_LIBRARIES})

##deps
#add_dependencies(pose_file_generator file_geneator_generate_messages_cpp)

