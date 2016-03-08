#include "ActiveSafety.h"

#include <cmath>

#include <bjos/libs/log.h>
#include <bjos/libs/geometry.h>

#include "modules/NearSpaceDetector.h"
#include "helpers/Potential.h"

void ActiveSafety::setMinimumDistanceInRange(double, double, double, double) {
    Log::fatal("ActiveSafety", "ActiveSafety::setMinimumDistanceInRange not implemented yet!");
}

void ActiveSafety::setRepulsionStrengthInRange(double, double, double, double) {
    Log::fatal("ActiveSafety", "ActiveSafety::setRepulsionStrengthInRange not implemented yet!");
}

void ActiveSafety::update() {
    //sync the near space minimum distance with the safety interface options
    _near_space_detector->setGlobalMinimumDistance(_active_safety_interface->getGlobalMinimumDistance());
    
    //update the near space
    _near_space_detector->update();
    
    //set default gradient to zero
    Vector gradient = {0, 0, 0};
    
    //handle (emergency) hold mode
    if(_active_safety_interface->holdEnabled()){
        //set gradient and forward to controller
        _controller_interface->setVelocity(gradient, AS_CTRL_VEL_FLAGS);
        _direction_gradient = gradient;
    
        _active_safety_interface->set_available(true);
        return;
    }
    
    //handle heading setpoint
    //FIXME: currently this enforces a hold setpoint at the same time, there is no support yet for rotating while flying (because this can be difficult...)
    if(_active_safety_interface->headingEnabled()){
        //set gradient and forward to controller
        _controller_interface->setYaw(_active_safety_interface->getTargetHeading());
        _direction_gradient = gradient;
        
        _active_safety_interface->set_available(true);
        return;
    }
    
    //get the last position of the controller
    Point current_position = _controller_interface->getPosition();
    double current_yaw = _controller_interface->getYaw();
    
    //get all potentials
    std::list<Potential> potentials = _near_space_detector->getPotentials();
    for(std::list<Potential>::iterator pot_iter = potentials.begin(); pot_iter != potentials.end(); ++pot_iter) {
        //TODO: check if in range
        //set repulsion strength (TODO: local strength)
        pot_iter->setStrength(getGlobalRepulsionStrength());
        Vector pot_gradient = pot_iter->getGradientOrigin();
        gradient = gradient + pot_gradient;
    }
    
    //handle next setpoint
    bool posMode = false;
    Vector target = getTargetPoint();
    if (std::isfinite(target.x()) && std::isfinite(target.y()) && std::isfinite(target.z())){        
        //get relative target
        Vector relative_target = target - current_position;
        
        //check if we should pos hold instead
        if(gradient.norm() < M_EPS && _controller_interface->hasWF()
            && relative_target.norm() < _active_safety_interface->getRadiusPositionMode()) posMode = true;
        
        //convert the target to body frame
        RotationMatrix rot(-current_yaw, 'z');
        relative_target = rot * relative_target;
        
        //make the attractive potential
        Potential target_pot(relative_target, new QuadraticLinearPotentialFunction(AS_POT_TRANS_RANGE), _target_attraction_strength);
        gradient = gradient + target_pot.getGradientOrigin();
    }
    
    //TODO: trigger the necessary events
    
    //set velocity zero if under minimum velocity (triggers position hold on pixhawk)
    if(fabs(std::sqrt(gradient.x()*gradient.x() + gradient.y()*gradient.y())) < _min_velocity_xy) gradient.x() = gradient.y() = 0;
    if(fabs(gradient.z()) < _min_velocity_z) gradient.z() = 0;
    
    
    //limit maximum velocity
    if(gradient.norm() > _max_velocity) gradient *= _max_velocity/gradient.norm();
        
    //set gradient
    _direction_gradient = gradient;
    
    //determine if we should send velocity or position
    if(posMode) _controller_interface->setPosition(target, _active_safety_interface->getControlFlags());
    else _controller_interface->setVelocity(gradient, _active_safety_interface->getControlFlags());

    
    //enable the active safety interface which has is now properly started up
    _active_safety_interface->set_available(true);
}

/*Point ActiveSafety::getDestination() {    
    Vector direction = _direction_gradient;
    
    //direction.normalize();
    
    direction.scale(_integration_length);
    return direction;
}*/

Vector ActiveSafety::getDirection() {
    return _direction_gradient;
}
