cmake_minimum_required(VERSION 3.0.2)
project(astar)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  genmsg
  message_generation
  nav_core
  tf
  rosbag
)

#add_message_files(
#   FILES
#)
#
#add_service_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/astar.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/astar.h
    )

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Hybrid A* Header and Source files
add_executable(astar src/astar.cpp ${SOURCES} ${HEADERS})

add_dependencies(astar ${PROJECT_NAME}_generate_messages_cpp )

target_link_libraries(astar ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)