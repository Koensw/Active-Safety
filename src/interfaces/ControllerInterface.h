#ifndef _BLUEJAY_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_CONTROLLER_INTERFACE_H_

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 */

class ControllerInterface: public SystemInterface{
public:
    /* Sets the target position for the controller */
    //FIXME: should be in local frame?
    //FIXME: cannot set yaw
    virtual void setPosition(Point) = 0;
    
    /* Gets the current position from the controller */
    virtual Point getPosition() = 0;
};

#endif