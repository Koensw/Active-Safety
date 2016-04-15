#ifndef _BLUEJAY_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_CONTROLLER_INTERFACE_H_

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 */

class ControllerInterface: public SystemInterface{
public:
    /* Sets the target velocity to the controller */
    virtual void setVelocity(Vector) = 0;
    virtual void setVelocity(Vector, uint32_t flags) = 0;
    
    /* Sets the target position to the controller */
    virtual void setPosition(Vector) = 0;
    virtual void setPosition(Vector, uint32_t flags) = 0;
    
    /* Gets the current position from the controller */
    virtual Point getPosition() = 0;
    virtual double getYaw() = 0;
    
    /* Check if the controller has a defined world frame */
    virtual bool hasWF() = 0;
    
    //TEMPORARY (FIXME)
    virtual void setYaw(double) {}
    virtual void setYaw(Vector, double) {}
};

#endif
