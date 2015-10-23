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
    BJOSControllerInterface(FlightController *controller): _controller(controller) {}
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setPosition(Point){
        //return - TODO: not yet implemented
    }
    Point getPosition(){
        return _controller->getPose().position;
    }
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setVelocity(Vector v){
        Heading head;
        head.velocity.vx = v.x;
        head.velocity.vy = v.y;
        head.velocity.vz = v.z;
        _controller->setTarget(SET_TARGET_VELOCITY, Pose(), head);
    }
private:
    FlightController *_controller;
    
    Point _current_position;
    Point _velocity;
};


#endif