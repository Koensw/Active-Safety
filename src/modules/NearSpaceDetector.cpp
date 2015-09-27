#include "NearSpaceDetector.h"

#include <cmath>

#include "log.h"

#include "helpers/Potential.h"
#include "sensors/DistanceSensor.h"

std::list<DistanceSensor*>::iterator NearSpaceDetector::registerSensor(DistanceSensor *sensor){
    //set maximum range that the sonar should detect
    //WARNING: this is local thus it could be that a unnecessary potential is spawned, this need to be corrected later
    sensor->setMaximumRange(_global_min_range);
    
    //add sensor
    _sensors.push_back(sensor);
    return --_sensors.end();
}

void NearSpaceDetector::unregisterSensor(std::list<DistanceSensor*>::iterator iter){
    //remove sensor
    delete *iter;
    _sensors.erase(iter);
}

//TODO: make this more clever to merge values from other potentials ??
double NearSpaceDetector::getDistanceAt(double yaw, double pitch){
    //find nearest potential and return distance
    std::list<Potential>::iterator nearest_potential = _potentials.end();
    double best_distance = 2*M_PI;
    for(std::list<Potential>::iterator pot_iter = _potentials.begin(); pot_iter != _potentials.end(); ++pot_iter){
        //calculate great circle distance and pick nearest
        std::pair<double, double> yaw_pitch = pot_iter->getYawPitch();
        double distance = acos(cos(pitch)*cos(yaw_pitch.second)*cos(yaw-yaw_pitch.first)+sin(pitch)*sin(yaw_pitch.second));
        
        if(distance < best_distance){
            best_distance = distance;
            nearest_potential = pot_iter;
        }
    }
    
    //TODO: trigger a warning if too far?
    
    if(nearest_potential == _potentials.end()) return _global_min_range;
    else return nearest_potential->getPosition().distanceOrigin();
}

double NearSpaceDetector::getMinimunDistanceInRange(double yawMin, double yawMax, double pitchMin, double pitchMax){
    double minimum_distance = _global_min_range;
    for(std::list<Potential>::iterator pot_iter = _potentials.begin(); pot_iter != _potentials.end(); ++pot_iter){
        std::pair<double, double> yaw_pitch = pot_iter->getYawPitch();
        
        //check if in range
        if(yawMin < yaw_pitch.first && yaw_pitch.first < yawMax && pitchMin < yaw_pitch.second && yaw_pitch.second < pitchMax){
            minimum_distance = std::min(minimum_distance, pot_iter->getPosition().distanceOrigin());
        }
    }
    return minimum_distance;
}

void NearSpaceDetector::update(){
    std::list<Potential> potentials;
    for(std::list<DistanceSensor*>::iterator sen_iter = _sensors.begin(); sen_iter != _sensors.end(); ++sen_iter){
        //retrieve local sensor potentials
        std::list<Potential> sensor_potentials = (*sen_iter)->getPotentials();

        //goto each potential and move them to correct location in body frame
        //remove all that are out of global range now
        for(std::list<Potential>::iterator pot_iter = sensor_potentials.begin(); pot_iter != sensor_potentials.end();){
            //rotate the potential to align the sensor and the body frame
            //DEBUG: std::cout << pot_iter->getPosition().x << " " << pot_iter->getPosition().y << " " << pot_iter->getPosition().z << std::endl;
            pot_iter->rotate(RotationMatrix(-(*sen_iter)->getPose().orientation));
            
            //translate the potential frame to the body frame
            pot_iter->translate((*sen_iter)->getPose().position);
            
            //check if we keep this potential or that is out of range now
            if(pot_iter->getPosition().distanceOrigin() > _global_min_range) pot_iter = sensor_potentials.erase(pot_iter);
            else ++pot_iter;
        }
        //TODO: trigger a potential event if this sensor did not had a potential before
                
        //merge it into the global list
        potentials.splice(potentials.end(), sensor_potentials);
    }

    //assign new potentials
    _potentials = potentials;
    
}

NearSpaceDetector::~NearSpaceDetector(){
    //unregister all left sensors
    while(!_sensors.empty()){
        unregisterSensor(_sensors.begin());
    }
}
