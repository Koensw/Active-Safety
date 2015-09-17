#include "SonarSensor.h"

#include "helpers/Potential.h"

std::list<Potential> SonarSensor::getPotentials(){
    //check if object in range
    if(getDistance() < _min_range) return std::list<Potential>();
    
    //make a negative potential
    Potential pot(-_repulsion_strength);
    
    std::list<Potential> pots;
    pots.push_back(pot);
    return pots;
}

double SonarSensor::getDistance(){
    return _interface->getDistance();
}