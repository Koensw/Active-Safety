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
    NearSpaceDetector(): _global_min_range(0) {} //default is to disable the near space
    NearSpaceDetector(double global_min_range): _global_min_range(global_min_range) {};
    
    /* Register and unregister a sensor in the space
     ALERT: the unregister takes care of deleting a DistanceSensor
     */
    std::list<DistanceSensor*>::iterator registerSensor(DistanceSensor*);
    void unregisterSensor(std::list<DistanceSensor*>::iterator);
    
    /* Returns all registered sensors */
    const std::list<DistanceSensor*> getSensors(){
        return _sensors;
    }
    
    /* Returns a single potential at the center of the FOV of the sonar at the reported distance */
    std::list<Potential> getPotentials(){
        return _potentials;
    }
    
    /* Gives the approximate distance at a given location */
    double getDistanceAt(double yaw, double pitch);
    /* Gives the minimum distance in a certain area */
    double getMinimunDistanceInRange(double yawMin, double yawMax, double pitchMin, double pitchMax);
    
    /* Set the global minimum distance to act on */
    void setGlobalMinimumDistance(double global_min_range);
    double getGlobalMinimumDistance(){
        return _global_min_range;
    }
    
    /* Updates the potentials with the last sensor information */
    void update();
    
    /* Unregisters all sensors */
    ~NearSpaceDetector();
private:
    //WARNING: disallow copy
    NearSpaceDetector(const NearSpaceDetector&);
    NearSpaceDetector& operator=(const NearSpaceDetector&);
    
    double _global_min_range;
    
    std::list<DistanceSensor*> _sensors;
    std::list<Potential> _potentials;
};

#endif
