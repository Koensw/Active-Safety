#ifndef _BLUEJAY_NEAR_SPACE_DETECTOR_H_
#define _BLUEJAY_NEAR_SPACE_DETECTOR_H_

#include <list>

#include "helpers/Potential.h"
#include "sensors/DistanceSensor.h"

/*
 * The near space detector provides access to obstacle detection in the near space of the copter
 * It has a list of potentials that represents approximate obstacles. 
 * It is used by the active safety layer to ensure that the copter does not bump into objects.
 */
class NearSpaceDetector{
public:
    NearSpaceDetector() {};
    
    /* Register and unregister a sensor in the space
     ALERT: the unregister takes care of deleting a DistanceSensor
     */
    std::list<DistanceSensor*>::const_iterator registerSensor(DistanceSensor*);
    void unregisterSensor(std::list<DistanceSensor*>::const_iterator);
    
    /* Returns all registered sensors */
    std::list<DistanceSensor*> getSensors(){
        return _sensors;
    }
    
    /* Returns a single potential at the center of the FOV of the sonar at the reported distance */
    std::list<Potential> getPotentials(){
        return _potentials;
    }
    
    /* Gives the approximate distance at a given location */
    double getDistanceAt(int hRad, int vRad);
    /* Gives the minimum distance in a certain area */
    double getMinimunDistanceInRange(int hRadMin, int vRadMin, int hRadMax, int vRadMax);
    
    /* Updates the potentials with the last sensor information */
    void update();
    
    /* Unregisters all sensors */
    ~NearSpaceDetector();
private:
    //WARNING: disallow copy
    NearSpaceDetector(const NearSpaceDetector&);
    NearSpaceDetector& operator=(const NearSpaceDetector&);
    
    std::list<DistanceSensor*> _sensors;
    std::list<Potential> _potentials;
};

#endif