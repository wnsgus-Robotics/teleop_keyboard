#include "velocity_profile.h"

VelocityProfile::VelocityProfile(double vacc,double wacc,double control_hz,double tolerance)
{
    this->control_hz = control_hz;
    this->vacc = vacc;
    this->wacc = wacc;
    this->tolerance = tolerance;
    curret_speed.linear.x = 0;
    curret_speed.angular.z = 0;
    ROS_INFO("[%s] control_hz: %f vacc: %f wacc: %f tolerance: %f",ros::this_node::getName().c_str(),control_hz,vacc,wacc,tolerance);
}

VelocityProfile::~VelocityProfile()
{

}

double VelocityProfile::calcVel(double current_speed,double acc)
{
    return current_speed + (acc * (1/control_hz));
}

geometry_msgs::Twist VelocityProfile::calc(geometry_msgs::Twist vel)
{
    target_speed = vel;
    if(fabs(curret_speed.linear.x - target_speed.linear.x) < tolerance)
        curret_speed.linear.x = target_speed.linear.x;
    else if(target_speed.linear.x > curret_speed.linear.x)
        curret_speed.linear.x = calcVel(curret_speed.linear.x,vacc);
    else if(target_speed.linear.x < curret_speed.linear.x)
        curret_speed.linear.x = calcVel(curret_speed.linear.x,-vacc);

    if(fabs(curret_speed.angular.z - target_speed.angular.z) < tolerance)
        curret_speed.angular.z = target_speed.angular.z;
    else if(target_speed.angular.z > curret_speed.angular.z)
        curret_speed.angular.z = calcVel(curret_speed.angular.z,wacc);
    else if(target_speed.angular.z < curret_speed.angular.z)
        curret_speed.angular.z = calcVel(curret_speed.angular.z,-wacc);
    return curret_speed;
}