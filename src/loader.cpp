/*
 * Startup code for the project, decides which mode to run 
 */
//#define ROS_MODE

//#ifdef ROS_MODE

#ifdef ROS_MODE
#include "ros/ros.h"
#include "interfaces/RosSonarInterface.h"
#include "interfaces/RosControllerInterface.h"
#endif

#include <chrono>
#include <thread>
#include <sstream>

#include "interfaces/SonarInterface.h"
#include "interfaces/BJOSControllerInterface.h"
#include "interfaces/BJOSSonarInterface.h"
#include "sensors/SonarSensor.h"
#include "modules/NearSpaceDetector.h"
#include "modules/ActiveSafety.h"

#include "potential_functions.h"

#include <bjos/bjos/bjos.h>
#include <bjos/bjos/helpers/process.h>

#include <bjos/libs/log.h>
#include <bjos/libs/i2c.h>
#include <bjos/libs/geometry.h>

#include <bjcomm/subscriber.h>
#include <bjcomm/message.h>

//include the currently used sensor model
#include "real_model.h"

using namespace bjos;
using namespace bjcomm;

std::vector<::SonarInterface*> sonar_interfaces;
ControllerInterface *controller_interface = 0;
ActiveSafetyInterface *active_safety_interface = 0;

SonarController *sonar_controller = 0;
FlightController *flight_controller = 0;

NearSpaceDetector *near_space_detector = 0;
ActiveSafety *active_safety = 0;

#ifdef ROS_MODE
#include "ros_loader.cpp"
#endif 

//init the BJOS
bool BJOSInit(int, char**){
    Log::info("ActiveSafetyLoader", "Initializing BJOS mode...");
    
    //load BJOS
    if(BJOS::getState() != BJOS::ACTIVE){
        Log::error("ActiveSafetyLoader", "Cannot continue, BJOS not active!");
        return false;
    }
    Process::installSignalHandler();
    BJOS *bjos = BJOS::getOS();
    
    //load the near space near space detector 
    near_space_detector = new NearSpaceDetector();
    
    //retrieve the sonar controller
    sonar_controller = new SonarController();
    if(!bjos->getController("sonar", sonar_controller)){
        Log::warn("ActiveSafetyLoader", "Current loader does not have sonar controller, not loading sonars!");
        return false;
    }else{
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
    }
    
    //retrieve the flight controller
    flight_controller = new FlightController();
    if(!bjos->getController("flight", flight_controller)){
        Log::error("ActiveSafetyLoader", "Cannot continue, current loader does not have flight controller!");
        return false;
    }
    
    //load the controller interface
    controller_interface = new BJOSControllerInterface(flight_controller);
    
    //wait for the controller and sensors interfaces to be available
    for(size_t i=0; i<sonar_interfaces.size(); ++i){
        while(Process::isActive() && !sonar_interfaces[i]->isAvailable()) {
            //TODO: implement a time lib function
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    while(Process::isActive() && !controller_interface->isAvailable()){
        //TODO: implement a time lib function
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    //load the active safety interface
    active_safety_interface = new ActiveSafetyInterface();
    
    //load the active safety
    active_safety = new ActiveSafety(near_space_detector, controller_interface, active_safety_interface);
    active_safety->setGlobalMinimumDistance(0.5);
    
    Log::info("ActiveSafetyLoader", "Finished initialization");
    return true;
}

//run the module
void BJOSRun(){
    //active_safety->setTargetPoint(Point(2, 2, 1));
    active_safety->setGlobalRepulsionStrength(0.5);
    active_safety->setTargetAttractionStrength(0.8);
    
    active_safety->setMinimumVelocityXY(0.05);
    active_safety->setMinimumVelocityZ(0.08);
    active_safety->setMaximumVelocity(0.8);
    
    //active_safety->setGlobalMinimumDistance(0.5);
    //active_safety->setGlobalMinimumDistance(10);

    active_safety->setRadiusPositionMode(100);
    
    Publisher status_pub("status");
    status_pub.start();
    while(Process::isActive()){
        /*for(size_t i=0; i<sonar_interfaces.size(); ++i){
            Log::info("ActiveSafetyLoader", "Distance %#1x %f", 0x70+i, sonar_interfaces[i]->getDistance());
        }*/

        //get current position from controller
        Point cur = controller_interface->getPosition();
        float yaw = controller_interface->getYaw();
//        Log::info("ActiveSafetyLoader", "Position %f %f %f --- Direction: %f %f %f", cur.x, cur.y, cur.z);;
        
        //update active safety
        active_safety->update();
        
        //get direction where flying to
        Vector direction = active_safety->getDirection();
        Log::info("ActiveSafetyLoader", "Position %f %f %f %f --- Direction %f %f %f", cur.x(), cur.y(), cur.z(), yaw, direction.x(), direction.y(), direction.z());
        Point tar = active_safety->getTargetPoint();
        Log::info("ActiveSafetyLoader", "Target %f %f %f", tar.x(), tar.y(), tar.z());
        
        Vector dir_wf = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * direction;
        std::ostringstream oss;
        oss << dir_wf.x() << " " << dir_wf.y() << " " << dir_wf.z() << std::endl;
        status_pub.send(Message("control_direction", oss.str()));
        
        //TODO: implement a time lib function
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

//finalize the BJOS
void BJOSFinalize(){
    Log::info("ActiveSafetyLoader", "Switch back to hold...");
    if(active_safety && active_safety_interface){
        active_safety_interface->setHold(true);
        active_safety->update();
    }

    Log::info("ActiveSafetyLoader", "Finalizing...");

    //delete interfaces
    for(size_t i=0; i<sonar_interfaces.size(); ++i){
        delete sonar_interfaces[i];
    }
    delete controller_interface;
    delete active_safety_interface;
    
    //delete bjos controllers
    delete flight_controller;
    delete sonar_controller;

    //delete near space and active safety
    delete near_space_detector;
    delete active_safety;
    
    Log::info("ActiveSafetyLoader", "Finished finalization");
}

int main(int argc, char **argv){  
#ifdef ROS_MODE
    bool ret = rosInit(argc, argv);
    if(ret) rosRun();
    rosFinalize();
#else
    bool ret = BJOSInit(argc, argv);
    if(ret) BJOSRun();
    BJOSFinalize();
#endif
}
