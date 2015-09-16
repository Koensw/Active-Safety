/*
 * Startup code for the project, decides which mode to run 
 */
#define ROS_MODE

#ifdef ROS_MODE
#include "ros/ros.h"
#endif

#include "interfaces/RosSonarInterface.h"
#include "interfaces/RosControllerInterface.h"

#include "log.h"
#include "geometry.h"

void ros_init(int argc, char **argv){
    Log::info("Loading ROS");
    
    ros::init(argc, argv, "active_safety_simulation");
}

void ros_run(){
    //load the sensor model
    RosSonarInterface rsi("sonar1");
    
    RosControllerInterface con("position");
    con.setPosition(Point(2, 4, 2));
    
    //loop until request to die
    ros::Rate rate(10);
    while(ros::ok()){
        Log::info("Distance %f", rsi.getDistance());
        Point cur = con.getPosition();
        Log::info("Position %f %f %f", cur.x, cur.y, cur.z);
        con.sync();
        
        rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv){
    Log::info("Starting");
    
#ifdef ROS_MODE
    ros_init(argc, argv);
    ros_run();
#else
    Log::fatal("Only supporting ROS atm..");
#endif
}