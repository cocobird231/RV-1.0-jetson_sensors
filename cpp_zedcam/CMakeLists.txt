cmake_minimum_required(VERSION 3.5)
project(cpp_zedcam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DROS_DISTRO=1)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_compile_options(-DROS_DISTRO=2)
else()# eloquent
  add_compile_options(-DROS_DISTRO=0)
endif()

# find dependencies
SET(ENABLE_TS_NODE False CACHE BOOL "Some user-specified option")
if( ENABLE_TS_NODE ) 
  add_compile_options(-DTS_NODE)
endif()

SET(ENABLE_QOS_NODE False CACHE BOOL "Some user-specified option")
if( ENABLE_QOS_NODE ) 
  add_compile_options(-DQOS_NODE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_interfaces REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

SET(SPECIAL_OS_LIB "pthread")

## Build
add_executable(pub src/main.cpp)
ament_target_dependencies(pub rclcpp vehicle_interfaces)
target_link_libraries(pub
  ${ZED_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}
)

## Install
install(TARGETS
  pub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
