#include "TestSonarInterface.h"

#include "geometry.h"

#include "ros/ros.h"

TestSonarInterface::TestSonarInterface(){
    //TODO: also set other fields
    _range = 0;
}

void TestSonarInterface::setDistance(double distance){
    if(!isAvailable()) set_available(true);
    _range = distance;
}

double TestSonarInterface::getDistance(){
    return _range;
}