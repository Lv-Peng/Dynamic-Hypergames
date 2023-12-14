cmake_minimum_required(VERSION 3.0.2)
project(DSO_EX_1104)
set(CMAKE_CXX_STANDARD 14)
#add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(OpenCV_DIR /usr/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  roscpp
  rospy
  std_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES DSO_EX_1104
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    /usr/include/compatibility/cpp_c_headers/
    /usr/include/eigen3
    /usr/include/yaml-cpp
  )

  add_executable(DSO src/MultiAgentMotion.cpp)
  target_link_libraries(DSO ${catkin_LIBRARIES})
  add_executable(ViconMeasure src/Measure.cpp)
  target_link_libraries(ViconMeasure ${catkin_LIBRARIES})
  add_executable(PointActuation_Single src/PointActuation_single.cpp)
  target_link_libraries(PointActuation_Single ${catkin_LIBRARIES})
  add_executable(PointActuation_Multi src/PointActuation_multi.cpp)
  target_link_libraries(PointActuation_Multi ${catkin_LIBRARIES})
  add_executable(Calibration src/vicon_camera_calibration.cpp)
  target_link_libraries(Calibration /usr/lib/libyaml-cpp.a ${catkin_LIBRARIES})
#  add_executable(ImageCap src/imagecapture.cpp)
#  target_link_libraries(ImageCap  ${catkin_LIBRARIES})
  add_executable(UR_Control src/ur_test.cpp)
  target_link_libraries(UR_Control  ${catkin_LIBRARIES})
  add_executable(Regg src/regret_pub.cpp)
  target_link_libraries(Regg  ${catkin_LIBRARIES})
  add_executable(Reg_s src/re_slam.cpp)
  target_link_libraries(Reg_s  ${catkin_LIBRARIES})
