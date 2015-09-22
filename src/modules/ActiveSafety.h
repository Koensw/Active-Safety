#ifndef _BLUEJAY_ACTIVE_SAFETY_H_
#define _BLUEJAY_ACTIVE_SAFETY_H_

#include "geometry.h"

#include "modules/NearSpaceDetector.h"
#include "interfaces/ControllerInterface.h"
#include "interfaces/ActiveSafetyInterface.h"

/*
 * Active safety controller that builds a safe flight path for the controller without bumping into objects
 */
class ActiveSafety{
public:
    ActiveSafety(NearSpaceDetector *near_space_detector, ControllerInterface *controller_interface, ActiveSafetyInterface *active_safety_interface): 
    _near_space_detector(near_space_detector), _controller_interface(controller_interface), _active_safety_interface(active_safety_interface), _global_repulsion_strength(1), _target_attraction_strength(1) {};
    
    /* Set global range */
    void setGlobalMinimumDistance(double distance){
        _near_space_detector->setGlobalMinimumDistance(distance);
    }
    double getGlobalMinimumDistance(){
        return _near_space_detector->getGlobalMinimumDistance();
    }
    /* Set repulsion and target attraction */
    void setGlobalRepulsionStrength(double repulsion_strength){
        _global_repulsion_strength = repulsion_strength;
    }
    double getGlobalRepulsionStrength(){
        return _global_repulsion_strength;
    }
    void setTargetAttractionStrength(double attraction_strength){
        _target_attraction_strength = attraction_strength;
    }
    double getTargetAttractionStrength(){
        return _target_attraction_strength;
    }
    
    /* Set range and repulsion in range 
     * ALERT: not implemented yet
     */
    void setMinimumDistanceInRange(double yawMin, double yawMax, double pitchMin, double pitchMax);
    void setRepulsionStrengthInRange(double yawMin, double yawMax, double pitchMin, double pitchMax);
    
    /* Set radius for integrating the direction */
    /*void setDestinationRadius(double radius){
        _integration_length = radius;
    }
    double getDestinationRadius(){
        return _integration_length;
    }*/
    
    /* Get/sets target point
     WARNING: expects this in global frame
     */
    void setTargetPoint(Point point){
        _active_safety_interface->setTargetPosition(point);
    }
    Point getTargetPoint(){
        return _active_safety_interface->getTargetPosition();
    }
    
    /* Update active safety layer */
    void update();
    
    /* Get direction and destination */
    //Point getDestination();
    Vector getDirection();
    
    /* Set position */
    void setPose();
    
    /* Get repulsion strength 
     ALERT: not yet known how to use this, possibly not needed at all
     */
    //Vector getAggressiveness();
    
private:
    //WARNING: disallow copy
    ActiveSafety(const ActiveSafety&);
    ActiveSafety& operator=(const ActiveSafety&);
    
    NearSpaceDetector *_near_space_detector;
    ControllerInterface *_controller_interface;
    ActiveSafetyInterface *_active_safety_interface;
    
    double _global_repulsion_strength;
    double _target_attraction_strength;
    
    Vector _direction_gradient;
    //double _integration_length;
};

#endif