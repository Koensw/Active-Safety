#ifndef _BLUEJAY_POTENTIAL_H_
#define _BLUEJAY_POTENTIAL_H_

#include <utility>

#include "geometry.h"

/*
 * Represents a potential in a potential field
 */

class Potential{
public:
    Potential(Point pos, double strength): _strength(strength), _pos(pos) {}
    
    /* Get/sets the potential strength */
    void setStrength(double strength){
        _strength = strength;
    }
    double getStrength(){
        return _strength;
    }
    void setPosition(Point pos){
        _pos = pos;
    }
    Point getPosition(){
        return _pos;
    }
    
    /* Returns a pair containing the yaw (first) and pitch (second) of a potential from the origin */
    std::pair<double, double> getYawPitch();
    
    /* Translate and rotate (around origin) methods */
    void translate(Vector vec);
    void rotate(RotationMatrix rot);
private:
    double _strength;
    Point _pos;
};

#endif