cmake_minimum_required(VERSION 3.0.2)
project(fetch-it)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find OpenCV package
find_package(OpenCV 3.2.0 REQUIRED)

## Find catkin and any other dependency packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport 
)
## Declare a catkin package
catkin_package()

## Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add executables
add_executable(detect_ball src/DetectBall.cpp src/detect_main.cpp)
add_executable(collect_objects src/collect_main.cpp src/ObstacleAvoidance.cpp src/TurtleBot.cpp src/DetectBall.cpp)

## Add target link libraries
target_link_libraries(
  detect_ball
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
## Add target link libraries
target_link_libraries(
  collect_objects
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

 if(CATKIN_ENABLE_TESTING)
  set(CMAKE_BUILD_TYPE Debug)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
    fetch-it-test
    test/fetch-it-test.launch
    test/TurtleBot_test.cpp
    ## test/ObstacleAvoidance_test.cpp
    test/main_test.cpp
    ## test/DetectBall_test.cpp
    src/ObstacleAvoidance.cpp
    src/TurtleBot.cpp
    src/DetectBall.cpp
    src/ObstacleAvoidance.cpp
  )

  target_link_libraries(
     fetch-it-test
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
  )

  add_dependencies(
    fetch-it-test
    detect_ball
    collect_objects
    ${catkin_EXPORTED_TARGETS}
  )
 endif()