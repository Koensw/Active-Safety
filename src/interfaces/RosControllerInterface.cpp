#include "RosControllerInterface.h"

#include "geometry_msgs/Point.h"

#include "log.h"

RosControllerInterface::RosControllerInterface(std::string pub_topic, std::string rec_topic){
    _topic = pub_topic;
    
    //publish to position topic
    _pub_pos = _node.advertise<geometry_msgs::Point>(_topic, 10);
    
    //subscribe to px4 location topic
    //FIXME: only for debugging...
    _sub_pos = _node.subscribe(rec_topic, 10, &RosControllerInterface::update_position, this);
}

void RosControllerInterface::setPosition(Point loc){
    //ALERT: NOT IMPLEMENTED (IN ROS WE HAVE GROUND TRUTH)
}

void RosControllerInterface::setVelocity(Vector vel){
    //update target
    _velocity = vel;
    
    //check difference between target and current position
    /*Vector diff = tar - getPosition();
    Log::info("Difference %f %f %f", diff.x, diff.y, diff.z);
    //FIXME: define in range better
    if(diff.length() < 0.1){
        //dont send new position, we are already there
        return;
    }*/
    
    //publish it on ROS
    geometry_msgs::Point pos;
    pos.x = _velocity.x;
    pos.y = _velocity.y;
    pos.z = _velocity.z;
    
    _pub_pos.publish(pos);
}

void RosControllerInterface::update_position(const px4::vehicle_local_position &pos){
    //the controller is available when we receive positions
    if(!isAvailable()) set_available(true);
    
    _current_position.x = pos.x;
    _current_position.y = pos.y;
    _current_position.z = -pos.z;
}