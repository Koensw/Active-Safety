#include "SonarReader.h"

#include <inttypes.h>
#include <math.h>
#include <time.h>

#include "log.h"

std::list<PhysicalSonarInterface*>::iterator SonarReader::registerSonar(PhysicalSonarInterface *sonar){
    //add sonar
    _sonars.push_back(sonar);
    return --_sonars.end();
}

void SonarReader::unregisterSonar(std::list<PhysicalSonarInterface*>::iterator iter){
    _sonars.erase(iter);
}

void SonarReader::update(){
    //do not do anything without sensors
    if(_sonars.empty()) return;
    
    //get time
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    double seconds = spec.tv_sec + spec.tv_nsec / 1.0e9;
    
    if(_last_update == 0){
        //first time only send data
        
        //send global through first interface
        //FIXME: maybe better to send separately
        (*_sonars.begin())->globalReadDistance();
        
        //set last update
        _last_update = seconds;
    }else if(seconds - _last_update >= _seconds_between){        
        //read the values from the interfaces
        for(std::list<PhysicalSonarInterface*>::iterator iter = _sonars.begin(); iter != _sonars.end(); ++iter){
            (*iter)->updateDistance();
        }
        
        //send new
        (*_sonars.begin())->globalReadDistance();
        _last_update = seconds;
    }
}