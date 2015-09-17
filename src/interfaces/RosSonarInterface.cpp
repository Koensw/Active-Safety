#include "RosSonarInterface.h"

#include "log.h"

#include "ros/ros.h"

RosSonarInterface::RosSonarInterface(std::string topic){
    _topic = topic;
    _range = _field_of_view = _min_range = _max_range = 0;
    
    //subscribe to ROS sonar topic
    _sub_rng = _node.subscribe(_topic, 10, &RosSonarInterface::update_information, this);
}

double RosSonarInterface::getDistance(){
    return _range;
}

void RosSonarInterface::update_information(const sensor_msgs::Range& range){
    //the sonar is available when we receive positions
    if(!isAvailable()) set_available(true);
    
    _range = range.range;
    _min_range = range.min_range;
    _max_range = range.max_range;
    _field_of_view = range.field_of_view;
}
