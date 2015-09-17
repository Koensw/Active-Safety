#ifndef _BLUEJAY_TEST_MODEL_H_
#define _BLUEJAY_TEST_MODEL_H_

#include <string>

#include "geometry.h"

/* 
 * Returns the sensor model of the sonars
 * TODO: This implementation should properly read a model file
 */

struct SonarInfo{
    //FIXME: topic is not portable
    std::string topic;
    
    Pose pose;
    
    double x;
    double y;
    double z;
    
    double hRad;
    double vRad;
};

//NOTE: imported from iris.xacro in flight_control
std::vector<SonarInfo> getSonarModel(){
    std::vector<SonarInfo> sonars;
    SonarInfo sonar1;
    sonar1.topic = "sonar1";
    sonar1.pose.position.x = 0;
    sonar1.pose.position.y = 0.02;
    sonar1.pose.position.z = 0.1;
    sonar1.pose.orientation.p = 0;
    sonar1.pose.orientation.y = 0.5*M_PI;
    sonars.push_back(sonar1);
    
    return sonars;
}

#endif