cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-D__PX4_ROS)
add_definitions(-D__EXPORT=)
project(active_safety)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(px4 REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs  
#   geometry_msgs
#   mav_msgs
#   gazebo_msgs
#)

catkin_package(
  LIBRARIES active_safety
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs libmavconn
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
include_directories(libs/)
include_directories(src/)
include_directories(models/)
add_executable(active_safety
   src/main.cpp
   src/interfaces/RosSonarInterface.cpp
   src/interfaces/RosControllerInterface.cpp
   src/sensors/SonarSensor.cpp
   src/modules/NearSpaceDetector.cpp
)
target_link_libraries(active_safety ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(flying_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(active_safety main.cpp)
#target_link_libraries(control ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(flying_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(flying_test_node
#   ${catkin_LIBRARIES}
# )
