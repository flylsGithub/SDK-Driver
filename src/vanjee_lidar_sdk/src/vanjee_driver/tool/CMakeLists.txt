
cmake_minimum_required(VERSION 3.5)

project(vanjee_driver_viewer)

message(=============================================================)
message("-- Ready to compile tools")
message(=============================================================)

include_directories(${DRIVER_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(WIN32)
  cmake_policy(SET CMP0074 NEW)
  set(OPENNI_ROOT "C:\\Program Files\\PCL 1.11.1\\3rdParty\\OpenNI2")
  set(OPENNI_LIBRARY "${OPENNI_ROOT}\\Lib\\OpenNI2.lib")
  set(OPENNI_INCLUDE_DIRS "${OPENNI_ROOT}\\Include\\")
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Release)
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Debug)
endif()

find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

option(ENABLE_TRANSFORM                               "Enable transform functions"                                        OFF)
option(ENABLE_GTEST                                   "Enable gtest value"                                                OFF)
option(ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE    "Enable transform timestamp to datetime with local time zone"       OFF)
option(ENABLE_GRAVITY_ACCELERATION_REMOVE             "Enable Gravity acceleration remove"                                OFF)
option(FILTER_DLL_ENABLE                              "Enablement is required when using the WLR-760"                     OFF)
option(ENABLE_SERIAL_BLOCK_MODE                       "Create the serial port using block mode"                           OFF)

if(${ENABLE_TRANSFORM})
 add_definitions("-DENABLE_TRANSFORM")

 find_package(Eigen3 REQUIRED)
 include_directories(${EIGEN3_INCLUDE_DIR})
endif()

if(${ENABLE_GTEST})
 add_definitions("-DENABLE_GTEST")
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

find_package(vanjee_driver REQUIRED)
include_directories(${VANJEE_DRIVER_INCLUDE_DIRS})

if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
else()
    list(APPEND EXTERNAL_LIBS pcap)
endif()

if(PCL_FOUND)

if(${FILTER_DLL_ENABLE})

  get_filename_component(_VANJEE_FILTER_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
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

add_executable(vanjee_driver_viewer
               vanjee_driver_viewer.cpp)

target_link_libraries(vanjee_driver_viewer
                    ${EXTERNAL_LIBS}    
                    ${PCL_LIBRARIES}
                    ${VANJEE_DRIVER_LIBRARIES})

if((VANJEE_FILTER_LIB))
  target_link_libraries(vanjee_driver_viewer ${VANJEE_FILTER_LIB})
endif()

else()

message("PCL Not found! Can not compile vanjee_driver_viewer!")

endif()

install(TARGETS vanjee_driver_viewer
        RUNTIME DESTINATION /usr/bin)     
