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
    sonar1.pose.position.x = 0.25;
    sonar1.pose.position.y = 0.0;
    sonar1.pose.position.z = 0.0;
    sonar1.pose.orientation.p = 0;
    sonar1.pose.orientation.y = 0.0;
    sonars.push_back(sonar1);
    
    SonarInfo sonar2;
    sonar2.topic = "sonar2";
    sonar2.pose.position.x = 0.25;
    sonar2.pose.position.y = 0.0;
    sonar2.pose.position.z = 0.0;
    sonar2.pose.orientation.p = 0;
    sonar2.pose.orientation.y = 0.6;
    sonars.push_back(sonar2);
    
    SonarInfo sonar3;
    sonar3.topic = "sonar3";
    sonar3.pose.position.x = 0.0;
    sonar3.pose.position.y = 0.25;
    sonar3.pose.position.z = 0.0;
    sonar3.pose.orientation.p = 0;
    sonar3.pose.orientation.y = 1.5;
    sonars.push_back(sonar3);

    SonarInfo sonar4;
    sonar4.topic = "sonar4";
    sonar4.pose.position.x = -0.25;
    sonar4.pose.position.y = 0.0;
    sonar4.pose.position.z = 0.0;
    sonar4.pose.orientation.p = 0;
    sonar4.pose.orientation.y = 3.0;
    sonars.push_back(sonar4);
    
    SonarInfo sonar5;
    sonar5.topic = "sonar5";
    sonar5.pose.position.x = 0.25;
    sonar5.pose.position.y = 0.0;
    sonar5.pose.position.z = 0.0;
    sonar5.pose.orientation.p = 0;
    sonar5.pose.orientation.y = -0.6;
    sonars.push_back(sonar5);
    
    SonarInfo sonar6;
    sonar6.topic = "sonar6";
    sonar6.pose.position.x = 0.0;
    sonar6.pose.position.y = -0.25;
    sonar6.pose.position.z = 0.0;
    sonar6.pose.orientation.p = 0;
    sonar6.pose.orientation.y = -1.5;
    sonars.push_back(sonar6);
    
    return sonars;
}

#endif