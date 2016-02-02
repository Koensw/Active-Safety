cmake_minimum_required(VERSION 2.8.3)
project(active_safety)

# load eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# set required dependencies
find_package(Boost COMPONENTS atomic system thread signals filesystem REQUIRED)
include_directories(include ${Boost_INCLUDE_DIRS})

# set compiler and flags
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

# add libraries and build executable
include_directories(src/)
include_directories(libs/)
include_directories(models/)

# TODO: divide build types better
# check for ros mode compile
if(${ROS})
    find_package(px4 REQUIRED) 
    add_definitions(-D__PX4_ROS)
    add_definitions(-D__EXPORT=)
    add_definitions(-DROS_MODE)

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

    # set catkin settings
    catkin_package(
        LIBRARIES active_safety
        CATKIN_DEPENDS message_runtime roscpp std_msgs libmavconn
        DEPENDS system_lib Boost
    )

    add_executable(active_safety_ros
        src/loader.cpp
        src/interfaces/ActiveSafetyInterface.cpp
        src/interfaces/TestSonarInterface.cpp
        src/interfaces/RosSonarInterface.cpp
        src/interfaces/RosControllerInterface.cpp
        src/interfaces/DevantechSonarInterface.cpp
        src/helpers/Potential.cpp
        src/sensors/SonarSensor.cpp
        src/modules/NearSpaceDetector.cpp
        src/modules/ActiveSafety.cpp
    )
    # FIXME: zmq should not be necessary !!!
    target_link_libraries(active_safety_ros bjos bjcomm zmq ${catkin_LIBRARIES} ${Boost_LIBRARIES} rt pthread)
else()
    # normal non ros compile
    add_executable(active_safety
        src/loader.cpp
        src/interfaces/ActiveSafetyInterface.cpp
        src/interfaces/TestSonarInterface.cpp
        src/helpers/Potential.cpp
        src/sensors/SonarSensor.cpp
        src/modules/NearSpaceDetector.cpp
        src/modules/ActiveSafety.cpp
    )
    
    # FIXME: zmq should not be necessary !!!
    target_link_libraries(active_safety bjos bjcomm zmq ${Boost_LIBRARIES} rt pthread)
endif()

