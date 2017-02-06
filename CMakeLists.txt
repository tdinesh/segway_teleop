cmake_minimum_required(VERSION 2.8.3)
project(segway_teleop)

set(PKG_DEPS
  std_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  roscpp
  topic_tools
  joy
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

catkin_package(CATKIN_DEPENDS ${PKG_DEPS})

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(teleop_segway src/teleop_segway.cpp)

target_link_libraries(teleop_segway ${catkin_LIBRARIES})

install(TARGETS teleop_segway
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
