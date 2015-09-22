# Active Safety Component #
Contains the process implementing the active safety. This module ensures that the drone does not bump into obstacles in it near space 

## Setup ##
To run the current simulation, the following steps are needed (NOTE: no guarantee that this works directly...):

1. Follow the basic SITL setup from [here](https://pixhawk.org/dev/ros/sitl)
2. Download the ROS flight control module from [here](https://bitbucket.org/bluejayeindhoven/ros-flight-control) to the catkin workspace source folder
3. Download this repository to the catkin workspace source
4. Execute `catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7/` from the catkin build root
5. Execute `source devel_isolated/setup.bash`
6. From one terminal run `roslaunch flight_control spawn_gui.launch`
7. From another terminal go to the source directory of this repo and execute `./run.sh`

## Interfaces ##
Currently implements the following interfaces (with specified methods) to communicate with this module.
Each interface implements at least an isAvailable() method. For the implementations see /src/interfaces/.

* GlobalPositionInterface
    * setGlobalPosition(Point): updates the global position of the controller
* ActiveSafetyInterface
    * setTargetPosition(Point): sets the target location that it should fly to (while keeping a safe distance)
    * setGlobalMinimumDistance(double): sets the global minimum safe distance (in meter), within this the module should react to obstacles
    * setGlobalRepulsionStrength(double): sets how aggressive this module should react to obstacles (NOT YET EXACTLY DEFINED)

The current default implementation of the GlobalPositionInterface just forwards the position to the controller.

The following interfaces should only be used by the module itself to communicate with other modules.

* SonarInterface
* ControllerInterface

Currently the default Sonar and Controller interfaces are implemented to communicate with Gazebo using ROS.

## TODO ##
* Further tuning of the model
* Support better configuration (and determine what can be done online)
* Implement custom range and strength at different locations
* Implement module tests
* Implement events
* Communication and parallel setup of the sonars
* Communication with real controller