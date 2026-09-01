cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(vanjee_lidar_sdk)

#=======================================
# Compile Type (Release, Debug)
#=======================================
set(CMAKE_BUILD_TYPE "Release")

#=======================================
# Custom Point Type (XYZI, XYZIRT, XYZIRTT)
#=======================================
set(POINT_TYPE XYZIRT)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

option(DISABLE_PCAP_PARSE                             "Disable PCAP file parse"                                                 ON )
option(ENABLE_PCL_POINTCLOUD                          "Enable PCL Point Cloud"                                                  OFF)
option(ENABLE_TRANSFORM                               "Enable transform functions"                                              OFF)
option(ENABLE_GTEST                                   "Enable gtest value"                                                      OFF)
option(ENABLE_EPOLL_RECEIVE                           "Receive packets with epoll() instead of select()"                        OFF)
option(ENABLE_WAIT_IF_QUEUE_EMPTY                     "Enable waiting for a while in handle thread if the queue is empty"       OFF)
option(ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE    "Enable transform timestamp to datetime with local time zone"             OFF)
option(ENABLE_GRAVITY_ACCELERATION_REMOVE             "Enable Gravity acceleration remove"                                      OFF)
option(USE_TIME_WITH_FLOAT_TYPE_FLAG                  "Time Component Data Type Switch(OFF-timestamp(double), ON-time(float))"  OFF)
option(ENABLE_PRINT_LOG_WITH_ROS_LOG                  "Enable print with ros log (OFF-std::cout, ON-ROS_INFO / RCLCPP_INFO)"    OFF)
option(FILTER_DLL_ENABLE                              "Enablement is required when using the WLR-760"                           OFF)
option(ENABLE_SERIAL_BLOCK_MODE                       "Create the serial port using block mode"                                 OFF)

if(${DISABLE_PCAP_PARSE})

  message(=============================================================)
  message("-- Disable PCAP parse")
  message(=============================================================)

  add_definitions("-DDISABLE_PCAP_PARSE")

else()

  if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
  else()
    list(APPEND EXTERNAL_LIBS pcap)
  endif()

endif()

if(${ENABLE_PCL_POINTCLOUD})
  add_definitions("-DENABLE_PCL_POINTCLOUD")
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
endif()

if(${ENABLE_TRANSFORM})
 add_definitions("-DENABLE_TRANSFORM")

 find_package(Eigen3 REQUIRED)
 include_directories(${EIGEN3_INCLUDE_DIR})
endif()

if(${ENABLE_GTEST})
 add_definitions("-DENABLE_GTEST")
endif()

if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif()

if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif()

if(${ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE})
  add_definitions("-DENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE")
endif()

if(${ENABLE_GRAVITY_ACCELERATION_REMOVE})
  add_definitions("-DENABLE_GRAVITY_ACCELERATION_REMOVE")
endif()

if(${USE_TIME_WITH_FLOAT_TYPE_FLAG})
  add_definitions("-DUSE_TIME_WITH_FLOAT_TYPE_FLAG")
endif()

if(${ENABLE_PRINT_LOG_WITH_ROS_LOG})
  add_definitions("-DENABLE_PRINT_LOG_WITH_ROS_LOG")
endif()

if(${ENABLE_SERIAL_BLOCK_MODE})
  add_definitions(-DENABLE_SERIAL_BLOCK_MODE)
endif()
#========================
# Project details / setup
#========================
set(PROJECT_NAME vanjee_lidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  message("buile type: Release")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message("buile type: Debug")
else ()
  set(CMAKE_BUILD_TYPE "Release")
  message("buile type: Release")
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble" OR $ENV{ROS_DISTRO} STREQUAL "jazzy")  # the ros2 humble/jazzy requires c++17
add_definitions(-std=c++17)
else()
add_definitions(-std=c++14)
endif()

# add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZIRTT")
add_definitions(-DPOINT_TYPE_XYZIRTT)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
elseif(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
endif()

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Dependencies Setup
#========================

#ROS#
find_package(roscpp 1.12 QUIET)

if(roscpp_FOUND AND (${COMPILE_METHOD} STREQUAL "ORIGINAL" OR  ${COMPILE_METHOD} STREQUAL "CATKIN"))

  message(=============================================================)
  message("-- ROS Found. ROS Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS_FOUND)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml")
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml
      ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
      COPYONLY
    )
    message(STATUS "Using ROS1 package.xml")
  else()
    message(WARNING "package_ros1.xml not found, using default package.xml")
  endif()

  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  add_definitions(-DRUN_IN_ROS_WORKSPACE)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    roslib
    std_msgs
    message_generation)

    add_message_files(
      FILES
      DeviceCtrl.msg
      VanjeelidarPacket.msg
      VanjeeLidarMultiPacket.msg
      LidarParameterInterface.msg
    )

    generate_messages(
      DEPENDENCIES
      std_msgs)

  catkin_package(CATKIN_DEPENDS
    sensor_msgs 
    roslib
    std_msgs 
    message_runtime)

  include_directories( ${catkin_INCLUDE_DIRS}/ros_msg)

else()

  message(=============================================================)
  message("-- ROS Not Found. ROS Support is turned Off.")
  message(=============================================================)

endif()

#ROS2#
find_package(rclcpp QUIET)

if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On.")
  message(=============================================================)

  add_definitions(-DROS2_FOUND)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml")
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml
      ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
      COPYONLY
    )
    message(STATUS "Using ROS2 package.xml")
  else()
    message(WARNING "package_ros2.xml not found, using default package.xml")
  endif()

  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  # find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  # find_package(rosidl_typesupport_cpp REQUIRED)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  set(msg_files
    "msg/DeviceCtrl.msg"
    "msg/VanjeelidarPacket.msg"
    "msg/VanjeeLidarMultiPacket.msg"
    "msg/LidarParameterInterface.msg"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES builtin_interfaces std_msgs
  )

else()

  message(=============================================================)
  message("-- ROS2 Not Found. ROS2 Support is turned Off.")
  message(=============================================================)

endif()

#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)
MESSAGE(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")

#Driver core#
add_subdirectory(src/vanjee_driver)
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================

add_executable(vanjee_lidar_sdk_node
               node/vanjee_lidar_sdk_node.cpp
               src/manager/node_manager.cpp)

if((${COMPILE_METHOD} STREQUAL "ORIGINAL") OR (${COMPILE_METHOD} STREQUAL "CATKIN"))
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

if(${FILTER_DLL_ENABLE})

  message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86|i386|i686|AMD64|x86_64")
      message(STATUS "Target architecture: x86")
      set(_VANJEE_FILTER_LIB_NAME VanJeeLaser760Filter_linux_x86)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
      message(STATUS "Target architecture: ARM")
      set(_VANJEE_FILTER_LIB_NAME VanJeeLaser760Filter_linux_aarch64)
  else()
      message(FATAL_ERROR "Unsupported architecture for FILTER_DLL_ENABLE: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  # Prefer the package-local .so to avoid broken/unreadable copies under /usr/local/lib
  find_library(VANJEE_FILTER_LIB
      NAMES ${_VANJEE_FILTER_LIB_NAME}
      PATHS ${PROJECT_SOURCE_DIR}/src/vanjee_driver
      NO_DEFAULT_PATH
  )
  if(NOT VANJEE_FILTER_LIB)
    find_library(VANJEE_FILTER_LIB
        NAMES ${_VANJEE_FILTER_LIB_NAME}
        PATHS ${PROJECT_SOURCE_DIR}/src/vanjee_driver /usr/local/lib
    )
  endif()
  if(NOT VANJEE_FILTER_LIB)
    message(FATAL_ERROR "Cannot find lib${_VANJEE_FILTER_LIB_NAME}.so under ${PROJECT_SOURCE_DIR}/src/vanjee_driver")
  endif()
  message(STATUS "Found filter library: ${VANJEE_FILTER_LIB}")
  add_definitions(-DFILTER_DLL_ENABLE)
  target_link_libraries(vanjee_lidar_sdk_node ${VANJEE_FILTER_LIB})

endif()

target_link_libraries(vanjee_lidar_sdk_node
                      ${YAML_CPP_LIBRARIES}
                      ${vanjee_driver_LIBRARIES})

MESSAGE(STATUS "vanjee_driver_LIBRARIES = ${vanjee_driver_LIBRARIES}")

#Ros#
if(roscpp_FOUND AND (${COMPILE_METHOD} STREQUAL "ORIGINAL" OR  ${COMPILE_METHOD} STREQUAL "CATKIN"))

  target_link_libraries(vanjee_lidar_sdk_node
    ${ROS_LIBS})

  if(${COMPILE_METHOD} STREQUAL "CATKIN")

    install(TARGETS vanjee_lidar_sdk_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

    install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
    install(DIRECTORY param/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
    )
    install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )

  endif()

endif()

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  target_include_directories(vanjee_lidar_sdk_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generated_cpp>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(vanjee_lidar_sdk_node
    ${PROJECT_NAME}__rosidl_typesupport_cpp)

  ament_target_dependencies(vanjee_lidar_sdk_node 
    rclcpp 
    std_msgs 
    sensor_msgs)

  install(TARGETS
    vanjee_lidar_sdk_node
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    config
    param
    DESTINATION share/${PROJECT_NAME})

  ament_package()

endif()
