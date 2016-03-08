#ifndef _BLUEJAY_BJOS_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_BJOS_CONTROLLER_INTERFACE_H_

#include "config.h"

#include <bjos/libs/geometry.h>
#include <bjos/libs/log.h>

#include <bjos/controllers/FlightController.h>
#include "ControllerInterface.h"

using namespace bjos;

/* 
 * ROS implementation of a controller interface
 */

//TODO: currently not thread safe
class BJOSControllerInterface : public ControllerInterface{
public:
    BJOSControllerInterface(FlightController *controller): _controller(controller) {
	_takeoff = false;
        //FIXME: check for proper flightcontroller
        set_available(controller->isAvailable());
    }
    
    void setPosition(Vector pos){
        setPosition(pos, 0xffffffff);
    }
    void setPosition(Vector pos, uint32_t flags){
     	flags = validate_flags(flags);
        _controller->setTargetWF(AS_CTRL_POS_FLAGS & flags, pos, 0, 0);
    }
    void setVelocity(Vector vel){
        setVelocity(vel, 0xffffffff);
    }
    void setVelocity(Vector vel, uint32_t flags){
        flags = validate_flags(flags);
        _controller->setTargetCF(AS_CTRL_VEL_FLAGS & flags, vel, 0, 0);
    }
    
    //FIXME: should be integrated in functions above
    void setYaw(double yaw){
        _controller->setTargetCF(AS_CTRL_VEL_FLAGS & AS_CTRL_YAW_FLAGS, {0, 0, 0}, yaw, 0);
    }
    
    Point getPosition(){
        return _controller->getPositionWF();
    }
    double getYaw(){
        return _controller->getOrientationWF().z();
    }
    
    bool hasWF(){
        return _controller->isWFDefined();
    }
private:
    uint32_t validate_flags(uint32_t flags){
        double alt = getPosition().z();
        //force a land when below minimum control altitude (unless taking off...)
        if((flags & AS_CTRL_FLAG_IGN_TAKEOFF) && alt < AS_MIN_SAFE_ALT){
            Log::warn("BJOSControllerInterface", "Enforcing land - below safe altitude");
            flags = AS_CTRL_LAND_FLAGS;
        }
        //disallow take off when above the minimum altitude (above this we can always in assume an in-air state)
        if(!(flags & AS_CTRL_FLAG_IGN_TAKEOFF)){
            if(!_takeoff && alt > AS_MIN_SAFE_ALT){
                Log::warn("BJOSControllerInterface", "Ignoring takeoff - already in air");
                flags |= AS_CTRL_FLAG_IGN_TAKEOFF;
            }else _takeoff = true;
        }else _takeoff = false;
        
        return flags;
    }
    
    FlightController *_controller;
    
    bool _takeoff;

    Point _current_position;
    Point _velocity;
};


#endif
