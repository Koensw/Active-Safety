/*
 * Startup code for the project, decides which mode to run 
 */
#define ROS_MODE

#ifdef ROS_MODE
#include "ros/ros.h"
#endif

#include "interfaces/RosSonarInterface.h"
#include "interfaces/TestSonarInterface.h"
#include "interfaces/RosControllerInterface.h"
#include "interfaces/PhysicalSonarInterface.h"
#include "interfaces/BJOSSonarInterface.h"
#include "helpers/SonarReader.h"
#include "sensors/SonarSensor.h"
#include "modules/NearSpaceDetector.h"
#include "modules/ActiveSafety.h"

#include "potential_functions.h"

#include <bjos/bjos/bjos.h>
#include <bjos/bjos/helpers/process.h>

#include "i2c.h"
#include "log.h"
#include "geometry.h"

//include the currently used sensor model
#include "real_model.h"

using namespace bjos;

std::vector<SonarInterface*> sonar_interfaces;
ControllerInterface *controller_interface = 0;
ActiveSafetyInterface *active_safety_interface = 0;

SonarController *sonar_controller;

NearSpaceDetector *near_space_detector = 0;
ActiveSafety *active_safety = 0;

//FIXME: TEMPORARY
PhysicalSonarInterface *phys_sonar;

//init the ROS simulation
bool rosInit(int argc, char **argv){
    Log::info("Initializing...");
    //init ROS
    ros::init(argc, argv, "active_safety_simulation");
    ros::start();
    
    //load BJOS
    if(BJOS::getState() != BJOS::ACTIVE){
        Log::error("Cannot continue, BJOS not active!");
        return false;
    }
    Process::installSignalHandler();
    BJOS *bjos = BJOS::getOS();
    
    //load the near space near space detector 
    near_space_detector = new NearSpaceDetector();
    
    //retrieve the sonar controller
    sonar_controller = new SonarController();
    if(!bjos->getController("sonar", sonar_controller)){
        Log::error("Cannot continue, current OS does not have sonar controllor!");
        return false;
    }
    
    //load the sensor model
    std::vector<SonarData> sonar_data = sonar_controller->getData();
    for(size_t i=0; i<sonar_data.size(); ++i){
        //create interface
        BJOSSonarInterface *sonar_interface = new BJOSSonarInterface(sonar_controller, sonar_data[i].id);
        sonar_interfaces.push_back(sonar_interface);
        
        //creates a sensor and register it
        SonarSensor *sonar_sensor = new SonarSensor(sonar_data[i].pose, sonar_interface);
        near_space_detector->registerSensor(sonar_sensor);
    }
    
    //load the controller interface
    controller_interface = new RosControllerInterface("velocity", "iris/vehicle_local_position");
    
    //wait for the controller and sensors interfaces to be available
    ros::Rate wait_init_rate(100);
    /*for(size_t i=0; i<sonar_interfaces.size(); ++i){
        while(Process::isActive() && !sonar_interfaces[i]->isAvailable()) {
            ros::spinOnce();
            wait_init_rate.sleep();
        }
    }*/
    while(Process::isActive() && !controller_interface->isAvailable()){
        ros::spinOnce();
        wait_init_rate.sleep();
    }
    
    //load the active safety interface
    active_safety_interface = new ActiveSafetyInterface();
    
    //load the active safety
    active_safety = new ActiveSafety(near_space_detector, controller_interface, active_safety_interface);
    active_safety->setGlobalMinimumDistance(0.5);
    
    Log::info("Finished initialization");
    return true;
}

//finalize the ROS simulation
void rosFinalize(){
    Log::info("Finalizing...");
    //delete interfaces
    for(size_t i=0; i<sonar_interfaces.size(); ++i){
        delete sonar_interfaces[i];
    }
    delete controller_interface;
    delete active_safety_interface;
    
    //delete bjos controllers
    delete sonar_controller;
    
    //delete near space and active safety
    delete near_space_detector;
    delete active_safety;
    
    ros::shutdown();
    Log::info("Finished finalization");
}

//run the simulation
void rosRun(){
    //set a position for the controller to goto
    //controller_interface->setTargetPosition(Point(0, 0, 4));
    
    //loop until request to die
    ros::Rate rate(50);
    int cnt = 0;
    
    //active_safety_interface->setTargetPosition(Point(8, 0, 3));
    active_safety->setTargetPoint(Point(0, 0, 3));
    
    //->setGlobalRepulsionStrength(5);
    active_safety->setGlobalRepulsionStrength(0.3);
    active_safety->setTargetAttractionStrength(0.2);
    while(Process::isActive()){
        for(size_t i=0; i<sonar_interfaces.size(); ++i){
            Log::info("Distance %#1x %f", 0x70+i, sonar_interfaces[i]->getDistance());
        }
        
        //get current position from controller
        Point cur = controller_interface->getPosition();
        Log::info("Position %f %f %f", cur.x, cur.y, cur.z);;
        
        //update active safety
        active_safety->update();
        
        //get direction where flying to
        Vector direction = active_safety->getDirection();
        Log::info("Direction %f %f %f", direction.x, direction.y, direction.z);
        
        rate.sleep();
        ros::spinOnce();
        
        ++cnt;
    }
}

void test(){
    
}

int main(int argc, char **argv){  
#ifdef ROS_MODE
    bool ret = rosInit(argc, argv);
    if(ret) rosRun();
    rosFinalize();
#else
    test();
    //Log::fatal("Only supporting ROS atm..");
#endif
}
