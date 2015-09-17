#ifndef _BLUEJAY_POTENTIAL_H_
#define _BLUEJAY_POTENTIAL_H_

#include "geometry.h"

/*
 * Represents a potential in a potential field
 */

class Potential{
public:
    Potential(double strength): _strength(strength) {}
    Potential(): _strength(0) {}
    
    /* Get/sets the potential strength */
    void setStrength(double strength){
        _strength = strength;
    }
    double getStrength(){
        return _strength;
    }
private:
    double _strength;
    Point _pos;
};

#endif