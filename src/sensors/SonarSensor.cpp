#include "SonarSensor.h"

#include "helpers/Potential.h"
#include "potential_functions.h"

std::list<Potential> SonarSensor::getPotentials(){
    //check if object in range
    if(getDistance() > _max_range) return std::list<Potential>();
    
    //get the position of the potential in the sensor frame
    Point pot_pos;
    pot_pos.y = pot_pos.z = 0;
    pot_pos.x = getDistance();
    
    //use a hyperbolic potential for the detection
    HyperbolicPotentialFunction *potential_function = new HyperbolicPotentialFunction();
    
    //create the potential
    Potential pot(pot_pos, potential_function);
    
    //add it to the single element list
    std::list<Potential> pots;
    pots.push_back(pot);
    return pots;
}

double SonarSensor::getDistance(){
    return _interface->getDistance();
}