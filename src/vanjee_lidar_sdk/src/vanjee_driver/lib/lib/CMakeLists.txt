cmake_minimum_required(VERSION 3.5)

project(vanjee_lidar_driver_lib)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message(=============================================================)
message("-- Ready to compile library")
message(=============================================================)

option(DISABLE_PCAP_PARSE                             "Disable PCAP file parse"                                           ON )
option(ENABLE_PCL_POINTCLOUD                          "Enable PCL Point Cloud"                                            OFF)
option(ENABLE_TRANSFORM                               "Enable transform functions"                                        OFF)
option(ENABLE_GTEST                                   "Enable gtest value"                                                OFF)
option(ENABLE_EPOLL_RECEIVE                           "Receive packets with epoll() instead of select()"                  OFF)
option(ENABLE_WAIT_IF_QUEUE_EMPTY                     "Enable waiting for a while in handle thread if the queue is empty" OFF)
option(ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE    "Enable transform timestamp to datetime with local time zone"       OFF)
option(ENABLE_GRAVITY_ACCELERATION_REMOVE             "Enable Gravity acceleration remove"                                OFF)
option(FILTER_DLL_ENABLE                              "Enablement is required when using the WLR-760"                     OFF)
option(ENABLE_SERIAL_BLOCK_MODE                       "Create the serial port using block mode"                           OFF)

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
  message(=============================================================)
  message("-- ENABLE_PCL_POINTCLOUD")
  message(=============================================================)
  add_definitions("-DENABLE_PCL_POINTCLOUD")
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

if(${ENABLE_TRANSFORM})
 add_definitions("-DENABLE_TRANSFORM")

 find_package(Eigen3 REQUIRED)
 include_directories(${EIGEN3_INCLUDE_DIR})
endif()

if(${ENABLE_GTEST})
 add_definitions("-DENABLE_GTEST")
endif(${ENABLE_GTEST})

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

if(${ENABLE_SERIAL_BLOCK_MODE})
  add_definitions(-DENABLE_SERIAL_BLOCK_MODE)
endif()

find_package(yaml-cpp REQUIRED)

find_package(vanjee_driver REQUIRED)
include_directories(${VANJEE_DRIVER_INCLUDE_DIRS})

add_library(vanjee_lidar_driver_lib SHARED
            vanjee_lidar_driver_lib.cpp)

if(${FILTER_DLL_ENABLE})

  get_filename_component(_VANJEE_FILTER_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
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
      PATHS ${_VANJEE_FILTER_LIB_DIR}
      NO_DEFAULT_PATH
  )
  if(NOT VANJEE_FILTER_LIB)
    find_library(VANJEE_FILTER_LIB
        NAMES ${_VANJEE_FILTER_LIB_NAME}
        PATHS ${_VANJEE_FILTER_LIB_DIR} /usr/local/lib
    )
  endif()
  if(NOT VANJEE_FILTER_LIB)
    message(FATAL_ERROR "Cannot find lib${_VANJEE_FILTER_LIB_NAME}.so under ${_VANJEE_FILTER_LIB_DIR}")
  endif()
  message(STATUS "Found filter library: ${VANJEE_FILTER_LIB}")
  add_definitions(-DFILTER_DLL_ENABLE)

endif()

target_link_libraries(vanjee_lidar_driver_lib 
                      ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})

if((VANJEE_FILTER_LIB))
  target_link_libraries(vanjee_lidar_driver_lib ${VANJEE_FILTER_LIB})
endif()
