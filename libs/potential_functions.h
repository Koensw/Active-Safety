#ifndef _BLUEJAY_POTENTIALS_H_
#define _BLUEJAY_POTENTIALS_H_

#include "geometry.h" 

#define POT_INFINITY std::numeric_limits<double>::infinity()

/* 
 * Implements a variety of potential functions used
 */

/* Base class */
class PotentialFunction{
public:
    /* Clone potential function */
    virtual PotentialFunction* clone() = 0;
    
    /* Get the value and the gradient of a potential
     * NOTE: getValue is possibly not needed at all */
    virtual double getValue(double r) = 0;
    virtual double getGradient(double r) = 0;
    
    /* Virtual destructor */
    virtual ~PotentialFunction(){}
};

/* Hyperbolic potential - useful for repulsive obstacles */
class HyperbolicPotentialFunction : public PotentialFunction{
public:
    PotentialFunction* clone(){
        return new HyperbolicPotentialFunction();
    }
    
    double getValue(double r){
        if(r < M_EPS) return POT_INFINITY;
        return 1/r;
    }
    double getGradient(double r){
        if(r < M_EPS) return POT_INFINITY;
        return -1/(r*r);
    }
};

/* Lineair quadratic attraction potential - useful for simple target points (nearer quadratic correction is wanted) */
class LinearPotentialFunction : public PotentialFunction{
public:    
    PotentialFunction* clone(){
        return new LinearPotentialFunction();
    }
    
    //TODO: implement transition range to quadratic
    double getValue(double r){
        return r;
    }
    double getGradient(double r){
        return 1;
    }
};

#endif 