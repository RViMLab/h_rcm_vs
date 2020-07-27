#pragma once

#include <vector>
#include <string.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <rcom_msgs/rcom.h>
#include <rcom_msgs/task.h>
#include <rcom_msgs/p_trocar.h>
#include <rcom_msgs/rcomAction.h>


class HRCoMVSActionClient
{
private:
    ros::NodeHandle _nh;

    rcom_msgs::task _task;
    rcom_msgs::p_trocar _p_trocar;

    actionlib::SimpleActionClient<rcom_msgs::rcomAction> _ac;

    ros::Timer _timer;

    // Subscribers
    ros::Subscriber _dtwist_sub;
    ros::Subscriber _p_trocar_sub;

    // Callbacks
    void _dTwistCB(const geometry_msgs::TwistConstPtr& dtwist_msg);
    void _pTrocarCB(const geometry_msgs::PoseConstPtr& p_trocar_msg);
    void _timerCB(const ros::TimerEvent&);


public:
    HRCoMVSActionClient(
        ros::NodeHandle& nh, 
        std::string& action_server, std::string& dtwist_topic, std::string& p_trocar_topic, 
        double dt
    );
    ~HRCoMVSActionClient();
};
