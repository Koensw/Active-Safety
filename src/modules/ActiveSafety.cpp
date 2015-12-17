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
    double current_yaw = _controller_interface->getYaw();
    
    //get all potentials
    Vector gradient;
    std::list<Potential> potentials = _near_space_detector->getPotentials();
    for(std::list<Potential>::iterator pot_iter = potentials.begin(); pot_iter != potentials.end(); ++pot_iter){
        //TODO: check if in range
        //set repulsion strength (TODO: local strength)
        pot_iter->setStrength(getGlobalRepulsionStrength());
        Vector pot_gradient = pot_iter->getGradientOrigin();
        gradient = gradient + pot_gradient;
    }
    
    //convert the target to local frame
    Point relative_target = getTargetPoint() - current_position;
    //convert the target to body frame
    RotationMatrix Rz(-current_yaw, 'z');
    relative_target = Rz.rotatePoint(relative_target);
    
    //make the attractive potential
    //FIXME: configure near space behaviour
    Potential target_pot(relative_target, new QuadraticLinearPotentialFunction(1), _target_attraction_strength);
    gradient = gradient + target_pot.getGradientOrigin();
    
    //TODO: trigger the necessary events
    
    //set velocity zero if under minimum velocity (triggers position hold on pixhawk)
    if(fabs(gradient.x) < _min_velocity_xy) gradient.x = 0;
    if(fabs(gradient.y) < _min_velocity_xy) gradient.y = 0;
    if(fabs(gradient.z) < _min_velocity_z) gradient.z = 0;
    
    //limit maximum velocity
    if(gradient.length() > _max_velocity) gradient.scale(_max_velocity/gradient.length());
    
    //set gradient and forward to controller
    _controller_interface->setVelocity(gradient);
    _direction_gradient = gradient;
    
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
