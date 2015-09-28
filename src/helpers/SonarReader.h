#ifndef _BLUEJAY_SONAR_READER_H_
#define _BLUEJAY_SONAR_READER_H_

#include <list>

#include "interfaces/PhysicalSonarInterface.h"

/*
 * Helper class that triggers the sonar read and updates and ensures proper timing
 */

//FIXME: better start a separate thread for this ?
class SonarReader{
public:
    SonarReader(): _seconds_between(1), _last_update(0) {}
    SonarReader(double seconds_between): _seconds_between(seconds_between), _last_update(0) {}
    
    /* Register and unregister sonar */
    std::list<PhysicalSonarInterface*>::iterator registerSonar(PhysicalSonarInterface *);
    void unregisterSonar(std::list<PhysicalSonarInterface*>::iterator);

    /* Set/get time between */
    void setSecondsBetween(double seconds_between){
        _seconds_between = seconds_between;
    }
    double getSecondsBetween(){
        return _seconds_between;
    }
    
    /* Reads and updates the registered sonars */
    void update();
private:
    double _seconds_between;
    double _last_update;
    
    std::list<PhysicalSonarInterface*> _sonars;
};

#endif