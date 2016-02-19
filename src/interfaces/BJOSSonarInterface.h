#ifndef _BLUEJAY_BJOS_SONAR_INTERFACE_H_
#define _BLUEJAY_BJOS_SONAR_INTERFACE_H_

#include "interfaces/SonarInterface.h"

#include <bjos/controllers/SonarController.h>

/* 
 * Base class for sonar interfaces that use real physical sonars
 */

//TODO: not threadsafe currently
class BJOSSonarInterface : public ::SonarInterface{
public:
    BJOSSonarInterface(bjos::SonarController *controller, int id):
        _controller(controller), _id(id) 
    {
        if(controller != nullptr) set_available(true);
    }
    
    double getDistance(){
        return _controller->getData(_id).distance;
    }
    
    /* Getter for specifications */
    double getFieldOfView(){
        return _controller->getData(_id).field_of_view;
    }
    double getMinRange(){
        return _controller->getData(_id).min_range;
    }
    double getMaxRange(){
        return _controller->getData(_id).max_range;
    }
private:
    bjos::SonarController *_controller;
    int _id;
protected:
    double _range;
};


#endif
