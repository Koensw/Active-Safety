#ifndef _BLUEJAY_PHYSICAL_SONAR_INTERFACE_H_
#define _BLUEJAY_PHYSICAL_SONAR_INTERFACE_H_

#include "SonarInterface.h"

/* 
 * Base class for sonar interfaces that use real physical sonars
 */

//TODO: not threadsafe currently
class PhysicalSonarInterface : public SonarInterface{
public:
    PhysicalSonarInterface(double fov, double min_range, double max_range): 
    _field_of_view(fov), _min_range(min_range), _max_range(max_range) {};
    
    /* Read, update and get distance */
    virtual void readDistance() = 0;
    virtual void updateDistance() = 0;
    double getDistance(){
        return _range;
    }

    /* Global static simulatenous update */
    //FIXME: this should actually have been static but that is impossible for virtual members...
    virtual void globalReadDistance() = 0;
    
    /* Getter and setters for arguments */
    double getFieldOfView(){
        return _field_of_view;
    }
    double getMinRange(){
        return _min_range;
    }
    double getMaxRange(){
        return _max_range;
    }
private:
    double _field_of_view;
    double _min_range;
    double _max_range;
protected:
    double _range;
};


#endif