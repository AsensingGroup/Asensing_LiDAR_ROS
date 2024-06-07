cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(aglidar_sdk)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZIRT)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD COLCON)

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")

  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_DOUBLE_RCVBUF "Enable double size of RCVBUF" ON)
if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})


#========================
# Project details / setup
#========================
set(PROJECT_NAME aglidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
add_definitions(-std=c++17)
add_definitions(-DPCL_USE_PCDIO)   #Ubuntu 22 need to use /pcd_io.h instead of /io.h
else()
add_definitions(-std=c++14)
endif()

add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
endif()

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Dependencies Setup
#========================

#ROS#
find_package(roscpp 1.12 QUIET)

if(roscpp_FOUND AND $ENV{ROS_VERSION} STREQUAL "1")

  message(=============================================================)
  message("-- ROS Found. ROS Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS1_FOUND)

  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  #Catkin#
  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    add_definitions(-DRUN_IN_ROS1_OR_ROS2)
    add_definitions(-DRUN_IN_ROS_WORKSPACE)

    find_package(catkin REQUIRED COMPONENTS
      roscpp
      sensor_msgs
      roslib)

    catkin_package(CATKIN_DEPENDS 
      sensor_msgs 
      roslib)

  endif(${COMPILE_METHOD} STREQUAL "CATKIN")

else(roscpp_FOUND AND $ENV{ROS_VERSION} STREQUAL "1")

  message(=============================================================)
  message("-- ROS Not Found. ROS Support is turned Off.")
  message(=============================================================)

endif(roscpp_FOUND AND $ENV{ROS_VERSION} STREQUAL "1")

#ROS2#
find_package(rclcpp QUIET)

if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On.")
  message(=============================================================)

  add_definitions(-DRUN_IN_ROS1_OR_ROS2)
  add_definitions(-DROS2_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(aglidar_msg REQUIRED)
  find_package(std_msgs REQUIRED)                      

else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Not Found. ROS2 Support is turned Off.")
  message(=============================================================)

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
option(COMPILE_DEMOS "Build lidar driver demos" OFF)
option(COMPILE_TOOLS "Build lidar driver tools" OFF)

add_subdirectory(src/driver_sdk)
find_package(ag_driver REQUIRED)
include_directories(${ag_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================

add_executable(aglidar_sdk_node
               node/aglidar_sdk_node.cpp
               src/manager/node_manager.cpp)

target_link_libraries(aglidar_sdk_node                   
                      ${YAML_CPP_LIBRARIES}
                      ${ag_driver_LIBRARIES})

#Ros#
if(roscpp_FOUND)

  target_link_libraries(aglidar_sdk_node 
    ${ROS_LIBS})

  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    install(TARGETS aglidar_sdk_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  endif()

endif(roscpp_FOUND)

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  ament_target_dependencies(aglidar_sdk_node 
    rclcpp 
    std_msgs 
    sensor_msgs 
    aglidar_msg)

  install(TARGETS
    aglidar_sdk_node
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})

  ament_package()

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

