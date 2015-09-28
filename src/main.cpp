/*
 * Startup code for the project, decides which mode to run 
 */
#define ROS_MODE
#define RPI_MODE

#ifdef ROS_MODE
#include "ros/ros.h"
#endif

#include "interfaces/RosSonarInterface.h"
#include "interfaces/TestSonarInterface.h"
#include "interfaces/RosControllerInterface.h"
#include "interfaces/DevantechSonarInterface.h"
#include "helpers/SonarReader.h"
#include "sensors/SonarSensor.h"
#include "modules/NearSpaceDetector.h"
#include "modules/ActiveSafety.h"

#include "potential_functions.h"

#include "i2c.h"
#include "log.h"
#include "geometry.h"

//include the currently used sensor model
#include "real_model.h"

std::vector<SonarInterface*> sonar_interfaces;
ControllerInterface *controller_interface = 0;
ActiveSafetyInterface *active_safety_interface = 0;

NearSpaceDetector *near_space_detector;
ActiveSafety *active_safety;

//start a sonar reader
SonarReader sonar_reader;

//FIXME: TEMPORARY
PhysicalSonarInterface *phys_sonar;

//init the ROS simulation
void rosInit(int argc, char **argv){
    Log::info("Initializing...");
    //init ROS
    ros::init(argc, argv, "active_safety_simulation");
    ros::start();
    
    //load the near space near space detector 
    near_space_detector = new NearSpaceDetector();
    
    //set the sonar reader helper
    sonar_reader.setSecondsBetween(0.2);
    
    //load the sensor model
    //TODO: properly read this from model file
    std::vector<SonarInfo> sensors = getSonarModel();
    for(size_t i=0; i<sensors.size(); ++i){
        //SonarInterface *sonar_interface = new RosSonarInterface(sensors[i].topic);
        PhysicalSonarInterface *sonar_interface = new DevantechSonarInterface(sensors[i].address);
        sonar_interfaces.push_back(sonar_interface);
        sonar_reader.registerSonar(sonar_interface);
        
        //creates a sensor and register it
        SonarSensor *sonar_sensor = new SonarSensor(sensors[i].pose, sonar_interface);
        near_space_detector->registerSensor(sonar_sensor);
    }
    
    //load the controller interface
    controller_interface = new RosControllerInterface("velocity", "iris/vehicle_local_position");
    
    //wait for the controller and sensors interfaces to be available
    ros::Rate wait_init_rate(100);
    for(size_t i=0; i<sonar_interfaces.size(); ++i){
        while(ros::ok() && !sonar_interfaces[i]->isAvailable()) {
            ros::spinOnce();
            wait_init_rate.sleep();
        }
    }
    while(ros::ok() && !controller_interface->isAvailable()){
        ros::spinOnce();
        wait_init_rate.sleep();
    }
    
    //load the active safety interface
    active_safety_interface = new ActiveSafetyInterface();
    
    //load the active safety
    active_safety = new ActiveSafety(near_space_detector, controller_interface, active_safety_interface);
    active_safety->setGlobalMinimumDistance(1);
    
    Log::info("Finished initialization");
    
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
    active_safety->setTargetPoint(Point(8, 0, 3));
    
    //->setGlobalRepulsionStrength(5);
    active_safety->setGlobalRepulsionStrength(0.3);
    active_safety->setTargetAttractionStrength(0.8);
    while(ros::ok()){
        for(size_t i=0; i<sonar_interfaces.size(); ++i){
            Log::info("Distance %#1x %f", 0x70+i, sonar_interfaces[i]->getDistance());
        }
        
        //get current position from controller
        Point cur = controller_interface->getPosition();
        Log::info("Position %f %f %f", cur.x, cur.y, cur.z);
        
        //update sensors
        sonar_reader.update();
        
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
#ifdef RPI_MODE
    I2C::start("/dev/i2c-1");
#endif
    
#ifdef ROS_MODE
    rosInit(argc, argv);
    rosRun();
    rosFinalize();
#else
    test();
    //Log::fatal("Only supporting ROS atm..");
#endif
    
#ifdef RPI_MODE
    I2C::stop();
#endif
}