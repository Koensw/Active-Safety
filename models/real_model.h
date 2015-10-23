#ifndef _BLUEJAY_TEST_MODEL_H_
#define _BLUEJAY_TEST_MODEL_H_

#include <string>

#include <bjos/libs/geometry.h>

/* 
 * Returns the sensor model of the sonars
 * TODO: This implementation should properly read a model file
 */

struct SonarInfo{
    unsigned char address;
    
    Pose pose;
};

//NOTE: imported from iris.xacro in flight_control
std::vector<SonarInfo> getSonarModel(){
    std::vector<SonarInfo> sonars;
    SonarInfo sonar1;
    sonar1.address = 0x72;
    sonar1.pose.position.x = 0.0;
    sonar1.pose.position.y = 0.0;
    sonar1.pose.position.z = 0.0;
    sonar1.pose.orientation.p = 0;
    sonar1.pose.orientation.y = 1.57079632679;
    sonars.push_back(sonar1);
    
    SonarInfo sonar2;
    sonar2.address = 0x71;
    sonar2.pose.position.x = 0.0;
    sonar2.pose.position.y = 0.0;
    sonar2.pose.position.z = 0.0;
    sonar2.pose.orientation.p = 0;
    sonar2.pose.orientation.y = 0;
    sonars.push_back(sonar2);
    
    SonarInfo sonar3;
    sonar3.address = 0x70;
    sonar3.pose.position.x = 0.0;
    sonar3.pose.position.y = 0.0;
    sonar3.pose.position.z = 0.0;
    sonar3.pose.orientation.p = 0;
    sonar3.pose.orientation.y = -1.57079632679;
    sonars.push_back(sonar3);
    
    return sonars;
}

#endif
