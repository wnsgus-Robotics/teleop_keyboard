#ifndef __TELEOP_KEYBOARD_H__
#define __TELEOP_KEYBOARD_H__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "velocity_profile.h"
#include "twist_mux.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)
a : auto run

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

)";


class TeleopKeyboard
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Publisher pub;
  ros::Timer timer1;
  ros::Timer timer2;

  std::string topic_name;
  std::string sub_topic_name;
  
  double speed; // Linear velocity (m/s)
  double turn; // Angular velocity (rad/s)
  double x,y,z,th;
  char key;
  double control_hz;
  double vacc;
  double wacc;
  bool use_mux;
  
  void getParam();
  int getch();
  VelocityProfile *profile;
  TwistMux *mux;

  void t1Callback(const ros::TimerEvent&);
  void t2Callback(const ros::TimerEvent&);
  geometry_msgs::Twist twist;

public:
  TeleopKeyboard();
  ~TeleopKeyboard();
  void init();

};

#endif 