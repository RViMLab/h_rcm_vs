#pragma once

#include <vector>
#include <string.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <rcm_msgs/rcm.h>
#include <rcm_msgs/task.h>
#include <rcm_msgs/p_trocar.h>
#include <rcm_msgs/rcmAction.h>


class HRCMVSActionClient
{
public:

    HRCMVSActionClient(
        ros::NodeHandle& nh, 
        std::string& action_server, std::string& twist_topic, std::string& p_trocar_topic, 
        double dt
    );

    ~HRCMVSActionClient();

private:

    ros::NodeHandle _nh;

    rcm_msgs::task _task;
    rcm_msgs::p_trocar _p_trocar;

    actionlib::SimpleActionClient<rcm_msgs::rcmAction> _ac;

    ros::Timer _timer;

    // Subscribers
    ros::Subscriber _twist_sub;
    ros::Subscriber _p_trocar_sub;

    // Callbacks
    void _twistCB(const geometry_msgs::TwistConstPtr& twist_msg);
    void _pTrocarCB(const geometry_msgs::PoseConstPtr& p_trocar_msg);
    void _timerCB(const ros::TimerEvent&);
};
