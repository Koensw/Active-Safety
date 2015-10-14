#ifndef _BLUEJAY_SONAR_INTERFACE_H_
#define _BLUEJAY_SONAR_INTERFACE_H_

#include "SystemInterface.h"

/*
 * Interface to the sonar (input for this module)
 */

class SonarInterface : public SystemInterface{
public:
    /* Returns the distance of the sonar */
    virtual double getDistance() = 0;
    
    /* Returns several properties of the sonar */
    virtual double getFieldOfView() = 0;
    virtual double getMinRange() = 0;
    virtual double getMaxRange() = 0;
};

#endif