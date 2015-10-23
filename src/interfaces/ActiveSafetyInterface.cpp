#include "ActiveSafetyInterface.h"

#include <boost/thread.hpp>
#include <bjcomm/subscriber.h>
#include <bjcomm/poller.h>

using namespace bjcomm;

ActiveSafetyInterface::ActiveSafetyInterface(): _global_repulsion_strength(1), _minimum_range(0){
    //_thrd_running = true;
    _thrd = boost::thread(&ActiveSafetyInterface::update, this);    
}

void ActiveSafetyInterface::update(){
    Subscriber sub("tcp://*:5556");
    bool ret = sub.start();
    if(!ret) Log::error("ActiveSafetyInterface", "Failed to load communication");
            
    Poller poller;
    int SUBSCRIBER = poller.add(&sub);
    while(true){
        poller.poll();        
                
        if(poller.hasMsg(SUBSCRIBER)){
            Message msg;
            msg = sub.receive();
    
            if(msg.getType() == "position"){
                int x, y, z;
                msg.getStream() >> x >> y >> z;
                setTargetPosition(Point(x, y, z));
                std::cout << "Set position to (" << x << "," << y << "," << z << ")" << std::endl;
                //std::exit(0);
            }
        }
    }
}