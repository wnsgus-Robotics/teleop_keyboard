cmake_minimum_required(VERSION 2.8.3)
project(teleop_keyboard)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
std_srvs
diagnostic_msgs
geometry_msgs
move_base_msgs
rosplan_dispatch_msgs
rosplan_knowledge_msgs
rosplan_planning_system
semantic_navigation_msgs
actionlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uvbot_msgs semantic_navigation_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs diagnostic_msgs geometry_msgs move_base_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs rosplan_planning_system actionlib 
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(teleop_keyboard src/teleop_keyboard.cpp src/velocity_profile.cpp src/twist_mux.cpp src/interaction_button.cpp)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_dependencies(behavior_goto_place semantic_navigation_msgs_generate_messages_cpp
  ${behavior_goto_place_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rosplan_knowledge_msgs_generate_messages_cpp
)