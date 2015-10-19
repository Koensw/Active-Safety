cmake_minimum_required(VERSION 2.8.3)
#add_definitions(-D__PX4_ROS)
#add_definitions(-D__EXPORT=)
project(active_safety)

# set required dependencies
find_package(px4 REQUIRED) 
find_package(Boost COMPONENTS system thread signals filesystem REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  cmake_modules
  gazebo_msgs
  sensor_msgs
  mav_msgs
  libmavconn
  tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# set compiler and flags
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-g -std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
link_directories("/usr/lib/arm-linux-gnueabihf/")

# set catkin settings
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES active_safety
  CATKIN_DEPENDS message_runtime roscpp std_msgs libmavconn
  DEPENDS system_lib Boost
)

# add libraries and build executable
include_directories(libs/)
include_directories(src/)
include_directories(models/)
add_library(geometry
    libs/geometry.cpp
)
add_library(i2c
    libs/i2c.cpp
)
add_executable(active_safety
    src/main.cpp
    src/interfaces/ActiveSafetyInterface.cpp
    src/interfaces/TestSonarInterface.cpp
    src/interfaces/RosSonarInterface.cpp
    src/interfaces/RosControllerInterface.cpp
    src/interfaces/DevantechSonarInterface.cpp
    src/helpers/Potential.cpp
    src/helpers/SonarReader.cpp
    src/sensors/SonarSensor.cpp
    src/modules/NearSpaceDetector.cpp
    src/modules/ActiveSafety.cpp
)

# FIXME: zmq should not be necessary !!!
target_link_libraries(active_safety bjos bjcomm zmq i2c geometry ${catkin_LIBRARIES} ${Boost_LIBRARIES} rt)
