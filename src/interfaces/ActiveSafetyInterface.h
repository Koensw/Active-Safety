#ifndef _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_
#define _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_

#include "log.h"

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 * 
 */

class ActiveSafetyInterface: public SystemInterface{
public:
    ActiveSafetyInterface() {}
    
    /* Sets the target position in the global frame */
    /* NOTE: ARCHITECTURE FUNCTION */
    //FIXME: should use pose instead of position
    void setTargetPosition(Point target){
        _target = target;
    }
    
    Point getTargetPosition(){
        Log::info("TARGET %f %f %f", _target.x, _target.y, _target.z);
        return _target; 
    }
private:
    Point _target;
};

#endif