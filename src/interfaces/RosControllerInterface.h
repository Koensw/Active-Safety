#ifndef _BLUEJAY_ROS_CONTROLLER_INTERFACE_H_
#define _BLUEJAY_ROS_CONTROLLER_INTERFACE_H_

#include <string>

#include "ros/ros.h"
#include "px4/vehicle_local_position.h"

#include "geometry.h"

#include "ControllerInterface.h"

/* 
 * ROS implementation of a controller interface
 */

//TODO: currently not thread safe
class RosControllerInterface : public ControllerInterface{
public:
    RosControllerInterface(std::string, std::string);
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setPosition(Point);
    Point getPosition(){
        return _current_position;
    }
    
    /* NOTE: ARCHITECTURE FUNCTION */
    void setVelocity(Vector);
private:
    void update_position(const px4::vehicle_local_position &pt); 
    
    ros::NodeHandle _node;
    ros::Subscriber _sub_pos;
    ros::Publisher _pub_pos;
    std::string _topic;
    
    Point _current_position;
    Point _velocity;
};

#endif