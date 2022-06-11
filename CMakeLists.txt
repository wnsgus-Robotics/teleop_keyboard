cmake_minimum_required(VERSION 2.8.3)
project(teleop_keyboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(teleop_keyboard src/teleop_keyboard.cpp src/velocity_profile.cpp src/twist_mux.cpp)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
