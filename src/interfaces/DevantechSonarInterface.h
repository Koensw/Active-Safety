#ifndef _BLUEJAY_DEVANTECH_SONAR_INTERFACE_H_
#define _BLUEJAY_DEVANTECH_SONAR_INTERFACE_H_

/* 
 * Devantech Sonar interface that works with the real sonars
 */

#include "PhysicalSonarInterface.h"

/* special devantech range values */
const int DEVANTECH_FOV = 1.04719755;
const int DEVANTECH_MIN_RANGE = 0.03;
const int DEVANTECH_MAX_RANGE = 5;

const int DEVANTECH_ERROR_RANGE = 10;

//TODO: not threadsafe currently
class DevantechSonarInterface : public PhysicalSonarInterface{
public:
    DevantechSonarInterface(unsigned char address): 
        PhysicalSonarInterface(DEVANTECH_FOV, DEVANTECH_MIN_RANGE, DEVANTECH_MAX_RANGE), _address(address) {
            _range = DEVANTECH_MAX_RANGE;
        }
        
    /* Read, update and get distance */
    void readDistance();
    void updateDistance();

    /* Global static simulatenous update */
    void globalReadDistance();
private:
    unsigned char _address;
};


#endif