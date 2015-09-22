#ifndef _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_
#define _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_

#include "log.h"

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 * 
 */

class ActiveSafetyInterface: public SystemInterface{
    friend class ActiveSafety;
public:
    ActiveSafetyInterface(): _global_repulsion_strength(1), _minimum_range(0) {}
    
    /* Sets the target position in the global frame */
    /* NOTE: ARCHITECTURE FUNCTION */
    //FIXME: should use pose instead of position
    void setTargetPosition(Point target){
        _target = target;
    }
    Point getTargetPosition(){
        return _target; 
    }
    
    /* Sets the global strength of the repulsion */
    /* NOTE: ARCHITECTURE FUNCTION */
    void setGlobalRepulsionStrength(double repulsion_strength){
        _global_repulsion_strength = repulsion_strength;
    }
    double getGlobalRepulsionStrength(){
        return _global_repulsion_strength;
    }
    
    /* Sets the global strength of the repulsion */
    /* NOTE: ARCHITECTURE FUNCTION */
    void setGlobalMinimumDistance(double minimum_range){
        _minimum_range = minimum_range;
    }
    double getGlobalMinimumDistance(){
        return _minimum_range;
    }
private:
    Point _target;
    double _global_repulsion_strength;
    double _minimum_range;
};

#endif