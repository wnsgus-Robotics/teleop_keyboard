#ifndef __VELOCITY_PROFILE_H__
#define __VELOCITY_PROFILE_H__

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
class VelocityProfile
{
private:
    geometry_msgs::Twist target_speed;
    geometry_msgs::Twist curret_speed;
    double control_hz;
    double vacc;
    double wacc;
    double tolerance;

    double calcVel(double before_speed,double acc);
public:
    VelocityProfile(double vacc,double wacc,double control_hz=10.0,double tolerance = 0.05);
    ~VelocityProfile();
    geometry_msgs::Twist calc(geometry_msgs::Twist vel);

};

#endif