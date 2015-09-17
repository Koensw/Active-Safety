#ifndef _BLUEJAY_TEST_SONAR_INTERFACE_H_
#define _BLUEJAY_TEST_SONAR_INTERFACE_H_

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include "SonarInterface.h"

/* 
 * Implements a test sonar interface where you can hard-code the distance of a sonar
 */

class TestSonarInterface : public SonarInterface{
public:
    TestSonarInterface();
    double getDistance();
    
    /* TODO: returns something useful */
    double getFieldOfView(){
        return 1/2*M_PI;
    }
    double getMinRange(){
        return 0;
    }
    double getMaxRange(){
        return 3;
    }
    
    /* Sets the distance that should be reported by the sonar */
    void setDistance(double);
private:
    double _range;
};

#endif