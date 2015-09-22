#ifndef _BLUEJAY_FORWARDING_GLOBAL_POSITION_INTERFACE_H_
#define _BLUEJAY_FORWARDING_GLOBAL_POSITION_INTERFACE_H_

#include "GlobalPositionInterface.h"

/*
 * Interface the controller (output of this module)
 */

class ForwardingGlobalPositionInterface: public GlobalPositionInterface{
public:
    ForwardingGlobalPositionController(ControllerInterface controller_interface) _controller_interface(controller_interface){
        //FIXME: not immediately available
        set_available(true);
    }
    
    /* Sets the target position for the controller */
    /* NOTE: ARCHITECTURE FUNCTION */
    virtual void setGlobalPosition(Point pos){
        //forward the position to the controller
        _controller_interface->setPosition(pos);
    }
    
    /* FIXME: forwarding the global position from the controller */
private
    ControllerInterface *_controller_interface;
};

#endif