#ifndef _BLUEJAY_SONAR_SENSOR_H_
#define _BLUEJAY_SONAR_SENSOR_H_

#include <list>

#include "helpers/Potential.h"
#include "interfaces/SonarInterface.h"

#include "DistanceSensor.h"

/*
 * Sonar sensor that spawns a single potential when something is in range
 */
class SonarSensor : public DistanceSensor{
public:
    SonarSensor(Pose pose, ::SonarInterface *interface): DistanceSensor(pose), _interface(interface) {};
    
    /* Returns a single potential at the center of the FOV of the sonar at the expected location */
    std::list<Potential> getPotentials();
    
    /* Returns the sonar distance */
    double getDistance();
private:
    //WARNING: disallow copy
    SonarSensor(const SonarSensor&);
    SonarSensor& operator=(const SonarSensor&);
    
    ::SonarInterface *_interface;
};

#endif