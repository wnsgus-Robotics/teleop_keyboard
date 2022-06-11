#ifndef __TWIST_MUX_H__
#define __TWIST_MUX_H__

#include <ros/ros.h>
#include <topic_tools/MuxSelect.h>
#include <topic_tools/MuxAdd.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <sstream>

class TwistMux
{
private:
    ros::NodeHandle nh;
	ros::NodeHandle pnh;

    ros::ServiceClient mux_sel_srv_;
    ros::Publisher cmd_vel_pub;

    std::string main_topic;
    std::string sub_topic;
    std::vector<std::string> topic_list;
    
    void init();
    void setService();
    void setParam();
    std::vector<std::string> split(std::string str,char token);

public:
    TwistMux(std::string main_topic,std::string sub_topic);
    ~TwistMux();
    
    bool select(std::string topic);
    std::string getMainTopic();
    std::vector<std::string> getTopicList();

};


#endif