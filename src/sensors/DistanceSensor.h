#ifndef _BLUEJAY_DISTANCE_SENSOR_H_
#define _BLUEJAY_DISTANCE_SENSOR_H_

#include <list>

#include "helpers/Potential.h"

/*
 *  Distance sensor used in the near space detector
 */
class DistanceSensor{
public:
    void setMinimumRange(double min_range){
        _min_range = min_range;
    }
    double getMinimumRange(){
        return _min_range;
    }
    void setRepulsionStrength(double repulsion_strength){
        _repulsion_strength = repulsion_strength;
    }
    double getRepulsionStrength(){
        return _repulsion_strength;
    }
    
    /* Returns a list of potentials in range for the sensor */
    virtual std::list<Potential> getPotentials() = 0;
protected:
    double _min_range;
    double _repulsion_strength;
};

#endif