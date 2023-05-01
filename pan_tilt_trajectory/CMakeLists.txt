cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pan_tilt_msgs
  message_runtime
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pan_tilt_msgs
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(trajPubNode
  src/traj_pub.cpp
)

target_link_libraries(trajPubNode
        ${catkin_LIBRARIES}
)

add_dependencies(trajPubNode pan_tilt_msgs_generate_messages_cpp)
