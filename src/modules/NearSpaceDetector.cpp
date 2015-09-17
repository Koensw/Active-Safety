#include "NearSpaceDetector.h"

#include "helpers/Potential.h"
#include "sensors/DistanceSensor.h"

std::list<DistanceSensor*>::const_iterator NearSpaceDetector::registerSensor(DistanceSensor *sensor){
    _sensors.push_back(sensor);
    return --_sensors.end();
}

void NearSpaceDetector::unregisterSensor(std::list<DistanceSensor*>::const_iterator iter){
    delete *iter;
    _sensors.erase(iter);
}

double NearSpaceDetector::getDistanceAt(int hRad, int vRad){
    
}

double NearSpaceDetector::getMinimunDistanceInRange(int hRadMin, int vRadMin, int hRadMax, int vRadMax){
    
}

void NearSpaceDetector::update(){
    
}

NearSpaceDetector::~NearSpaceDetector(){
    //unregister all left sensors
    while(!_sensors.empty()){
        unregisterSensor(_sensors.begin());
    }
}