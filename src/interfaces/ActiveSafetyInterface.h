#ifndef _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_
#define _BLUEJAY_ACTIVE_SAFETY_INTERFACE_H_

#include <bjos/libs/log.h>
#include <bjos/libs/geometry.h>
#include <bjos/controllers/FlightController.h> //FIXME: not portable?

#include <bjcomm/poller.h>

#include <boost/thread.hpp>
#include <atomic>

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
        _target = target;
    }
    Point getTargetPosition(){
        return _target; 
    }
    
    void setControlFlags(uint32_t flags){
        _flags = flags;
    }
    uint32_t getControlFlags(){
        return _flags;
    }
    
    /* Sets the global strength of the repulsion */
    /* NOTE: ARCHITECTURE FUNCTION */
    void setGlobalRepulsionStrength(double repulsion_strength){
        _global_repulsion_strength = repulsion_strength;
    }
    double getGlobalRepulsionStrength(){
        return _global_repulsion_strength;
    }
    
    /* Sets the global strength of the repulsion */
    /* NOTE: ARCHITECTURE FUNCTION */
    void setGlobalMinimumDistance(double minimum_range){
        _minimum_range = minimum_range;
    }
    double getGlobalMinimumDistance(){
        return _minimum_range;
    }
private:
    void update();
    
    bjcomm::Poller _poller;
    
    Point _target;
    double _global_repulsion_strength;
    double _minimum_range;
    
    uint32_t _flags;
    
    boost::thread _thrd;
    std::atomic_bool _thrd_running;
};

#endif
