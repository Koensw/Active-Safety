#ifndef _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_
#define _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_

#include <bjos/libs/log.h>
#include <bjos/libs/geometry.h>
#include <bjos/controllers/FlightController.h> //FIXME: not portable?

#include <bjcomm/poller.h>

#include <boost/thread.hpp>
#include <atomic>
#include <mutex>

#include "SystemInterface.h"

/*
 * Interface the controller (output of this module)
 * 
 */

class ActiveSafetyInterface: public SystemInterface{
    friend class ActiveSafety;
public:
    ActiveSafetyInterface();
    ~ActiveSafetyInterface(){
        _poller.interrupt();
        _thrd.interrupt();
        _thrd.join();
    }
    
    /* Sets the target position in the global frame */
    /* NOTE: ARCHITECTURE FUNCTION */
    //FIXME: should use pose instead of position
    void setTargetPosition(Point target){
        std::lock_guard<std::mutex> lock(_mutex);
        _target = target;
    }
    Point getTargetPosition(){
        std::lock_guard<std::mutex> lock(_mutex);
        return _target; 
    }
    
    void setControlFlags(uint32_t flags){
        std::lock_guard<std::mutex> lock(_mutex);
        _flags = flags;
    }
    uint32_t getControlFlags(){
        std::lock_guard<std::mutex> lock(_mutex);
        return _flags;
    }

    bool holdEnabled() {
        std::lock_guard<std::mutex> lock(_mutex);
        return _hold;
    }
    void setHold(bool hold){
        std::lock_guard<std::mutex> lock(_mutex);
        _hold = hold;
    }
    
    //FIXME: should be properly integrated in to target
    void setTargetHeading(double hdg){
        std::lock_guard<std::mutex> lock(_mutex);
        _heading = hdg;
    }
    double getTargetHeading(){
        return _heading;
    }
    bool headingEnabled(){
        std::lock_guard<std::mutex> lock(_mutex);
        return std::isfinite(_heading);
    }
    //FIXME: end
    
    /* Sets the global strength of the repulsion */
    /* NOTE: ARCHITECTURE FUNCTION */
    void setGlobalRepulsionStrength(double repulsion_strength){
        std::lock_guard<std::mutex> lock(_mutex);
        _global_repulsion_strength = repulsion_strength;
    }
    double getGlobalRepulsionStrength(){
        std::lock_guard<std::mutex> lock(_mutex);
        return _global_repulsion_strength;
    }
    
    /* Sets the global strength of the repulsion */
    /* NOTE: ARCHITECTURE FUNCTION */
    void setGlobalMinimumDistance(double minimum_range){
        std::lock_guard<std::mutex> lock(_mutex);
        _minimum_range = minimum_range;
    }
    double getGlobalMinimumDistance(){
        std::lock_guard<std::mutex> lock(_mutex);
        return _minimum_range;
    }
    
    /* Sets the radius where in position mode is used instead of velocity control whenever possible */
    void setRadiusPositionMode(double radius){
        std::lock_guard<std::mutex> lock(_mutex);
        _position_radius = radius;
    }
    double getRadiusPositionMode(){
        std::lock_guard<std::mutex> lock(_mutex);
        return _position_radius;
    }
private:
    void update();
    
    bjcomm::Poller _poller;
    
    std::mutex _mutex;
    Point _target;
    std::atomic_bool _hold;
    
    //FIXME: this should be part of the target (not a separate mode)
    double _heading;

    double _global_repulsion_strength;
    double _minimum_range;
    double _position_radius;
    
    uint32_t _flags;
    
    boost::thread _thrd;
    std::atomic_bool _thrd_running;
};

#endif
