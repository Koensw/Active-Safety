#include "SonarSensor.h"

#include "helpers/Potential.h"

std::list<Potential> SonarSensor::getPotentials(){
    //check if object in range
    if(getDistance() > _min_range) return std::list<Potential>();
    
    //get the position of the potential in the sensor frame
    Point pot_pos;
    pot_pos.y = pot_pos.z = 0;
    pot_pos.x = getDistance();
    
    //make a negative potential at the position
    Potential pot(pot_pos, -_repulsion_strength);
    
    //add it to the single element list
    std::list<Potential> pots;
    pots.push_back(pot);
    return pots;
}

double SonarSensor::getDistance(){
    return _interface->getDistance();
}