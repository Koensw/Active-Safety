#ifndef _BLUEJAY_DISTANCE_SENSOR_H_
#define _BLUEJAY_DISTANCE_SENSOR_H_

#include <list>

#include <bjos/libs/geometry.h>

#include "helpers/Potential.h"

/*
 *  Distance sensor used in the near space detector
 */
class DistanceSensor{
public:
    DistanceSensor(Pose pose): _max_range(0), _pose(pose) {}
    
    /* Properties getters/setters */
    void setMaximumRange(double max_range){
        _max_range = max_range;
    }
    double getMaximumRange(){
        return _max_range;
    }
    void setPose(Pose pose){
        _pose = pose;
    }
    Pose getPose(){
        return _pose;
    }
    
    /* Returns a list of potentials in range for the sensor */
    virtual std::list<Potential> getPotentials() = 0;
    
    /* Virtual destructor */
    virtual ~DistanceSensor() {}
protected:
    //WARNING: disallow copy
    DistanceSensor(const DistanceSensor&);
    DistanceSensor& operator=(const DistanceSensor&);
    
    double _max_range;
    
    Pose _pose;
    
};

#endif