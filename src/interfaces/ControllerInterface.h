#ifndef _BLUEJAY_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_CONTROLLER_INTERFACE_H_

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 */

class ControllerInterface: public SystemInterface{
public:
    /* Sets the current updated position of the controller */
    //FIXME: cannot set yaw?
    virtual void setPosition(Point) = 0;
    
    /* Sets the target velocity to the controller */
    //FIXME: should do this in local frame ?
    //FIXME: we should set the velocities instead
    virtual void setVelocity(Vector) = 0;
    
    /* Gets the current position from the controller */
    virtual Point getPosition() = 0;
    virtual double getYaw() = 0;
};

#endif