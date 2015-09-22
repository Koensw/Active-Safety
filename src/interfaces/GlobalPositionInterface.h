#ifndef _BLUEJAY_GLOBAL_POSITION_INTERFACE_H_
#define _BLUEJAY_GLOBAL_POSITION_INTERFACE_H_

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 */

class GlobalPositionInterface: public SystemInterface{
public:
    /* Sets the target position for the controller */
    virtual void setGlobalPosition(Point) = 0;
    
    /* FIXME: get forwarding */
};

#endif