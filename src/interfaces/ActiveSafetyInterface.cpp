#include "ActiveSafetyInterface.h"

#include "config.h"

#include <cmath>

#include <boost/thread.hpp>
#include <bjcomm/subscriber.h>

using namespace bjcomm;

ActiveSafetyInterface::ActiveSafetyInterface(): _target(NAN, NAN, NAN), _hold(false), _heading(NAN), _global_repulsion_strength(AS_REP_STRENGTH), _minimum_range(AS_MIN_RANGE), 
_position_radius(AS_POS_RADIUS), _flags(AS_CTRL_FLAGS) {
    _thrd = boost::thread(&ActiveSafetyInterface::update, this);    
}

void ActiveSafetyInterface::update(){
    try{
        Subscriber sub("modules/active_safety");
        bool ret = sub.start();
        if(!ret) {
            Log::error("ActiveSafetyInterface", "Failed to load communication");
            return;
        }
        
        int SUBSCRIBER = _poller.add(&sub);
        while(true){
            _poller.poll();        
            
            boost::this_thread::interruption_point();
            
            if(_poller.hasMsg(SUBSCRIBER)){
                Message msg;
                msg = sub.receive();
                                
                if(msg.getType() == "position"){
                    double x, y, z;
                    uint32_t fls;
                    msg.getStream() >> x >> y >> z >> fls;
                    if(fls) setControlFlags(fls);
                    else setControlFlags(AS_CTRL_FLAGS);
                    setTargetPosition(Point(x, y, z));
                    setHold(false);
                    setTargetHeading(NAN);
                }
                // NOTE: DONT USE THIS WHEN POSITION ESTIMATES ARE OK - THIS WILL SIMPLY GIVE (0,0,0)
                // VELOCITY SETPOINTS SO WE CANNOT CORRECT DRIFT
                else if (msg.getType() == "hold") {
                    setHold(true);
                    setControlFlags(SET_TARGET_VELOCITY);
                }
                // TODO: should be looked more accurate how to fix this
                else if (msg.getType() == "heading"){
                     double y;
                     msg.getStream() >> y;
                     setHold(false);
                     setTargetPosition(Vector(NAN, NAN, NAN));
                     setTargetHeading(y);
                }else if(msg.getType() == "setting"){
                    std::string type;
                    double value;
                    msg.getStream() >> type >> value;
                    if(type == "repulse-range") setGlobalMinimumDistance(value);
                }
            }
        }
    }catch(boost::thread_interrupted){
        //nothing special to handle here
        return;
    }
}
