#ifndef _BLUEJAY_BJOS_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_BJOS_CONTROLLER_INTERFACE_H_

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
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setPosition(Point){
        //return - TODO: not yet implemented
    }
    Point getPosition(){
        return _controller->getPositionWF();
    }
    double getYaw(){
        return _controller->getOrientationWF().z();
        //return 0;
    }
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setVelocity(Vector vel){
        setVelocity(vel, 0xffffffff);
    }
    void setVelocity(Vector vel, uint32_t flags){
        //TODO: protect here for invalid flags...
        _controller->setTargetCF(SET_TARGET_VELOCITY & flags,Vector(),Vector(), vel, Vector(0,0,0));
    }
private:
    FlightController *_controller;
    
    Point _current_position;
    Point _velocity;
};


#endif
