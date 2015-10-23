#ifndef _BLUEJAY_POTENTIAL_H_
#define _BLUEJAY_POTENTIAL_H_

#include <utility>

#include <bjos/libs/geometry.h>

#include "potential_functions.h"

/*
 * Represents a potential in a potential field
 */

class Potential{
public:
    Potential(Point pos, PotentialFunction *potential_function, double strength = 1): 
        _strength(strength), _pos(pos), _potential_function(potential_function) {}
        
    /* Copy behaviour */
    Potential(const Potential &pot): _strength(pot._strength), _pos(pot._pos){
        _potential_function = pot._potential_function->clone();
    }
    Potential &operator =(const Potential &pot){
        _strength = pot._strength;
        _pos = pot._pos;
        delete _potential_function;
        _potential_function = pot._potential_function->clone();
        return *this;
    }
    
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
    
    /* Gets the force at the located point and origin */
    Vector getGradient(Point);
    Vector getGradientOrigin(){
        return getGradient(Point(0, 0, 0));
    }
    
    /* Returns a pair containing the yaw (first) and pitch (second) of a potential from the origin */
    std::pair<double, double> getYawPitch();
    
    /* Translate and rotate (around origin) methods */
    void translate(Vector vec);
    void rotate(RotationMatrix rot);
    
    /* Virtual destructors */
    ~Potential() {
        delete _potential_function;
    }
private:
    double _strength;
    Point _pos;
    
    PotentialFunction *_potential_function;
};

#endif