#include "RosControllerInterface.h"

#include "geometry_msgs/Point.h"

#include "log.h"

RosControllerInterface::RosControllerInterface(std::string topic){
    _topic = topic;
    
    //publish to position topic
    _pub_pos = _node.advertise<geometry_msgs::Point>(_topic, 10);
    
    //subscribe to px4 location topic
    //FIXME: only for debugging...
    _sub_pos = _node.subscribe("iris/vehicle_local_position", 10, &RosControllerInterface::update_position, this);
}

void RosControllerInterface::setPosition(Point tar){
    _target = tar;
    
    geometry_msgs::Point pos;
    pos.x = _target.x;
    pos.y = _target.y;
    pos.z = _target.z;
    _pub_pos.publish(pos);
}

void RosControllerInterface::update_position(const px4::vehicle_local_position &pos){
    //the controller is available when we receive positions
    if(!isAvailable()) set_available(true);
    
    _current.x = pos.x;
    _current.y = pos.y;
    _current.z = pos.z;
}