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
        //ALERT: we should NOT use NED here, change this when world frame is properly defined!
        Point position = _controller->getPoseWF().position;
        return position;
    }
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setVelocity(Vector v){
        Heading head;
        head.velocity.vx = v.x;
        head.velocity.vy = v.y;
        head.velocity.vz = v.z;
        _controller->setTargetCF(SET_TARGET_VELOCITY, Pose(), head);
    }
private:
    FlightController *_controller;
    
    Point _current_position;
    Point _velocity;
};


#endif
