#include "ActiveSafety.h"

#include <bjos/libs/log.h>
#include <bjos/libs/geometry.h>

#include "modules/NearSpaceDetector.h"
#include "helpers/Potential.h"

void ActiveSafety::setMinimumDistanceInRange(double, double, double, double){
    Log::fatal("ActiveSafety", "ActiveSafety::setMinimumDistanceInRange not implemented yet!");
}

void ActiveSafety::setRepulsionStrengthInRange(double, double, double, double){
    Log::fatal("ActiveSafety", "ActiveSafety::setRepulsionStrengthInRange not implemented yet!");
}

void ActiveSafety::update(){
    //sync the near space minimum distance with the safety interface options
    _near_space_detector->setGlobalMinimumDistance(_active_safety_interface->getGlobalMinimumDistance());
    
    //update the near space
    _near_space_detector->update();
    
    //get the last position of the controller
    Point current_position = _controller_interface->getPosition();

    //get all potentials
    Vector gradient;
    std::list<Potential> potentials = _near_space_detector->getPotentials();
    for(std::list<Potential>::iterator pot_iter = potentials.begin(); pot_iter != potentials.end(); ++pot_iter){
        //TODO: check if in range
        //set repulsion strength (TODO: local strength)
        pot_iter->setStrength(getGlobalRepulsionStrength());
        Vector pot_gradient = pot_iter->getGradientOrigin();
        gradient = gradient + pot_gradient;
	//Log::info("ActiveSafety", "POTENTIAL");
    }
    
    //convert the target to local frame
    Point relative_target = getTargetPoint() - current_position;
    
    //make the attractive potential
    //FIXME: configure near space behaviour
    Potential target_pot(relative_target, new QuadraticLinearPotentialFunction(1), _target_attraction_strength);
    gradient = gradient + target_pot.getGradientOrigin();
    
    //TODO: trigger the necessary events
    //_direction_gradient = gradient;
    
    //push to the controller
    Vector velocity = gradient;
    //Log::info("LENGTH: %f", velocity.length());
    //Log::info("TEST: ", "%f %f", _max_velocity, velocity.length());
    if(velocity.length() > _max_velocity) velocity.scale(_max_velocity/velocity.length());
    _controller_interface->setVelocity(velocity);
    
    //set gradient
    _direction_gradient = velocity;
    
    //enable the active safety interface which has is now properly started up
    _active_safety_interface->set_available(true);
}

/*Point ActiveSafety::getDestination(){    
    Vector direction = _direction_gradient;
    
    //direction.normalize();
    
    direction.scale(_integration_length);
    return direction;
}*/

Vector ActiveSafety::getDirection(){
    return _direction_gradient;
}

/*//WARNING: difference between this and function above depends on frame (see header)
Vector ActiveSafety::getAggressiveness(){
    return Vector();
}*/
