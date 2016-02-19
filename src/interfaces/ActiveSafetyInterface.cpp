#include "ActiveSafetyInterface.h"

#include "config.h"

#include <boost/thread.hpp>
#include <bjcomm/subscriber.h>

using namespace bjcomm;

ActiveSafetyInterface::ActiveSafetyInterface(): _global_repulsion_strength(AS_REP_STRENGTH), _minimum_range(AS_MIN_RANGE), _flags(AS_CTRL_FLAGS){
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
                    setTargetPosition(Point(x, y, z));
                    std::cout << "Set position to (" << x << "," << y << "," << z << ")" << std::endl;
                    //std::exit(0);
                }
            }
        }
    }catch(boost::thread_interrupted){
        //nothing special to handle here
        return;
    }
}
