#include "ActiveSafety.h"

#include "log.h"
#include "geometry.h"

#include "modules/NearSpaceDetector.h"
#include "helpers/Potential.h"

void ActiveSafety::setMinimumDistanceInRange(double yawMin, double yawMax, double pitchMin, double pitchMax){
    Log::fatal("ActiveSafety::setMinimumDistanceInRange not implemented yet!");
}

void ActiveSafety::setRepulsionStrengthInRange(double yawMin, double yawMax, double pitchMin, double pitchMax){
    Log::fatal("ActiveSafety::setRepulsionStrengthInRange not implemented yet!");
}

void ActiveSafety::update(){
    //update the near space
    _near_space_detector->update();
    
    //get all potentials
    Vector gradient;
    std::list<Potential> potentials = _near_space_detector->getPotentials();
    for(std::list<Potential>::iterator pot_iter = potentials.begin(); pot_iter != potentials.end(); ++pot_iter){
        //TODO: check if in range
        //set repulsion strength (TODO: local strength)
        pot_iter->setStrength(_global_repulsion_strength);
        gradient = gradient + pot_iter->getGradientOrigin();
    }
    
    //make the attractive potential
    Potential target_pot(_target_point, new LinearPotentialFunction(), _target_attraction_strength);
    gradient = gradient + target_pot.getGradientOrigin();
    
    //TODO: trigger the necessary events
    _direction_gradient = gradient;
}

Point ActiveSafety::getDestination(){
    Vector direction = _direction_gradient;
    direction.normalize();
    
    direction.scale(_integration_length);
    return direction;
}

Vector ActiveSafety::getDirection(){
    return _direction_gradient;
}

/*//WARNING: difference between this and function above depends on frame (see header)
Vector ActiveSafety::getAggressiveness(){
    return Vector();
}*/