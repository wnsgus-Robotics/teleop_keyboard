#ifndef __INTERACTION_BUTTON_H__
#define __INTERACTION_BUTTON_H__

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include "semantic_navigation_msgs/Task.h"
#include "semantic_navigation_msgs/DoorState.h"
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Empty.h>

class InteractionButton
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Publisher cancel_publisher;
    ros::Publisher elevator_door_publisher;
    ros::Publisher opened_elevator_publisher;


public:
    InteractionButton();
    ~InteractionButton();

    void in(char key);

    bool cancelMovebase(char key);
    bool doorSatae(char key);
    bool elevatorChoose(char key);

};

#endif