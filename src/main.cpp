/*
 * Startup code for the project, decides which mode to run 
 */
#define ROS_MODE

#ifdef ROS_MODE
#include "ros/ros.h"
#endif

#include "interfaces/RosSonarInterface.h"
#include "interfaces/RosControllerInterface.h"
#include "sensors/SonarSensor.h"
#include "modules/NearSpaceDetector.h"

//include the currently used sensor model
#include "test_model.h"

#include "log.h"
#include "geometry.h"

std::vector<SonarInterface*> sonar_interfaces;
ControllerInterface *controller_interface = nullptr;

NearSpaceDetector near_space_detector;

//init the ROS simulation
void rosInit(int argc, char **argv){
    Log::info("Loading ROS");
    //init ROS
    ros::init(argc, argv, "active_safety_simulation");
    ros::start();
    
    //load the sensor model
    //TODO: properly read this from model file
    std::vector<SonarInfo> sensors = getSonarModel();
    for(int i=0; i<sensors.size(); ++i){
        SonarInterface *sonar_interface = new RosSonarInterface(sensors[i].topic);
        sonar_interfaces.push_back(sonar_interface);
        
        //creates a sensor and register it
        SonarSensor *sonar_sensor = new SonarSensor(sonar_interface);
        near_space_detector.registerSensor(sonar_sensor);
    }
    
    controller_interface = new RosControllerInterface("position");
}

//finalize the ROS simulation
void rosFinalize(){
    for(int i=0; i<sonar_interfaces.size(); ++i){
        delete sonar_interfaces[i];
    }
    delete controller_interface;
    
    ros::shutdown();
}

//run the simulation
void rosRun(){
    //set a position for the controller to goto
    controller_interface->setPosition(Point(2, 4, 2));
    
    //loop until request to die
    ros::Rate rate(10);
    int cnt = 0;
    while(cnt < 100){
        //get distance from sonar and current position from controller
        Log::info("Distance %f", (*sonar_interfaces.begin())->getDistance());
        Point cur = controller_interface->getPosition();
        Log::info("Position %f %f %f", cur.x, cur.y, cur.z);
        
        rate.sleep();
        ros::spinOnce();
        
        ++cnt;
    }
}

int main(int argc, char **argv){
    Log::info("Starting");
    
#ifdef ROS_MODE
    rosInit(argc, argv);
    rosRun();
    rosFinalize();
#else
    Log::fatal("Only supporting ROS atm..");
#endif
}