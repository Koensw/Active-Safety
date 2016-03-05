#ifndef _BLUEJAY_BJOS_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_BJOS_CONTROLLER_INTERFACE_H_

#include "config.h"

#include <bjos/libs/geometry.h>

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
        //FIXME: check for proper flightcontroller
        set_available(true);
    }
    
    void setPosition(Vector pos){
        setPosition(pos, 0xffffffff);
    }
    void setPosition(Vector pos, uint32_t flags){
        //FIXME: protect here for invalid flags
        _controller->setTargetWF(AS_CTRL_POS_FLAGS & flags, pos, 0, 0);
    }
    
    double getYaw(){
        return _controller->getOrientationWF().z();
    }
    
    void setVelocity(Vector vel){
        setVelocity(vel, 0xffffffff);
    }
    void setVelocity(Vector vel, uint32_t flags){
        //TODO: protect here for invalid flags...
        _controller->setTargetCF(AS_CTRL_VEL_FLAGS & flags, Vector(), Vector(), vel, Vector(0,0,0));
    }
    
    Point getPosition(){
        return _controller->getPositionWF();
    }
    
    bool hasWF(){
        return _controller->isWFDefined();
    }
private:
    FlightController *_controller;
    
    Point _current_position;
    Point _velocity;
};


#endif
