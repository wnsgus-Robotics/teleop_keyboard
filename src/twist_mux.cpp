#include "twist_mux.h"

TwistMux::TwistMux(std::string main_topic,std::string sub_topic):pnh("~")
{
    this->main_topic = main_topic;
    this->sub_topic = sub_topic;
    topic_list.push_back(main_topic);
    topic_list.push_back(sub_topic);
    setService();
    setParam();
    init();
}

TwistMux::~TwistMux()
{
    
}

void TwistMux::setService()
{
    mux_sel_srv_ = nh.serviceClient<topic_tools::MuxSelect>("mux/select");
}

void TwistMux::setParam()
{

}

void TwistMux::init()
{
    ros::service::waitForService("mux/select");
}

std::vector<std::string> TwistMux::split(std::string str,char token) 
{
    std::vector<std::string> internal;
    std::stringstream ss(str);
    std::string temp;
 
    while (getline(ss,temp,token)) 
    {
        internal.push_back(temp);
    }
 
    return internal;
}

bool TwistMux::select(std::string topic)
{
    if (*find(topic_list.begin(),topic_list.end(),topic) == topic)
    {
        topic_tools::MuxSelect mux_sel_;
        mux_sel_.request.topic = topic;
        if(mux_sel_srv_.call(mux_sel_))
        {
            ROS_INFO("[%s] topic change %s",ros::this_node::getName().c_str(),topic.c_str());
            return true;
        }
        else
        {
            ROS_ERROR("[%s] %s topic change error",ros::this_node::getName().c_str(),topic.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("[%s] non-existent topic",ros::this_node::getName().c_str());
        return false;
    }
}

std::string TwistMux::getMainTopic()
{
    return main_topic;
}

std::vector<std::string> TwistMux::getTopicList()
{
    return topic_list;
}

