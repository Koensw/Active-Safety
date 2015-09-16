#ifndef _BLUEJAY_ROS_SONAR_INTERFACE_H_
#define _BLUEJAY_ROS_SONAR_INTERFACE_H_

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include "SonarInterface.h"

/* 
 * ROS implementation of a sonar interface
 */

//TODO: currently not thread safe
class RosSonarInterface : public SonarInterface{
public:
    RosSonarInterface(std::string);
    double getDistance();
    
    double getFieldOfView(){
        return _field_of_view;
    }
    double getMinRange(){
        return _min_range;
    }
    double getMaxRange(){
        return _max_range;
    }
private:
    void update_information(const sensor_msgs::Range &range);
    
    ros::NodeHandle _node;
    ros::Subscriber _sub_rng;
    std::string _topic;

    double _range;
    double _min_range;
    double _max_range;
    double _field_of_view;
};

#endif