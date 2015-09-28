#include "DevantechSonarInterface.h"

#include "geometry.h"
#include "i2c.h"

void DevantechSonarInterface::readDistance(){
    //request for a distance read in cm
    I2C::write(_address, 0, 0x51);
}

//NOTE: expects that a cm reading is requested (this should only be done by this class)
void DevantechSonarInterface::updateDistance(){
    //read distance
    unsigned int total = 0;
    unsigned char data;
    int ret_val;
    ret_val = I2C::read(_address, 2, data);
    if(ret_val < 0) {
        _range = DEVANTECH_ERROR_RANGE;
        return;
    }
    total = data;
    ret_val = I2C::read(_address, 3, data);
    if(ret_val < 0) {
        _range = DEVANTECH_ERROR_RANGE;
        return;
    }
    total <<= 8;
    total |= data;
    if(total == 0) _range = DEVANTECH_MAX_RANGE;
    else _range = total/100.0;
    
    //the sonar is is avaible now we have our first range
    if(!isAvailable()) set_available(true);    
}

void DevantechSonarInterface::globalReadDistance(){
    //request for a global distance read in cm
    I2C::write(0, 0, 0x51);
}