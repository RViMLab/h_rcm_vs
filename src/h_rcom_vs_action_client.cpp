#include <h_rcom_vs/h_rcom_vs_action_client.h>


// Callbacks
void HRCoMVSActionClient::_dTwistCB(const geometry_msgs::TwistConstPtr& dtwist_msg) {
    // Sets _task
    _task.values[0] = dtwist_msg->linear.x;
    _task.values[1] = dtwist_msg->linear.y;
    _task.values[2] = dtwist_msg->linear.z;
    _task.values[3] = dtwist_msg->angular.x;
    _task.values[4] = dtwist_msg->angular.y;
    _task.values[5] = dtwist_msg->angular.z;

    _task.is_velocity = true;
}


void HRCoMVSActionClient::_pTrocarCB(const geometry_msgs::PoseConstPtr& p_trocar_msg) {
    // Sets _p_trocar
    _p_trocar.position.x = p_trocar_msg->position.x;
    _p_trocar.position.y = p_trocar_msg->position.y;
    _p_trocar.position.z = p_trocar_msg->position.z;

    _p_trocar.is_empty = false;
}


void HRCoMVSActionClient::_timerCB(const ros::TimerEvent&) {
    // Send goal request to action server
    rcom_msgs::rcomGoal goal;

    goal.states.p_trocar = _p_trocar;
    goal.states.task = _task;

    _ac.sendGoal(goal);
}


// Constructor and destructor
HRCoMVSActionClient::HRCoMVSActionClient(ros::NodeHandle& nh, std::string& action_server, std::string& dtwist_topic, std::string& p_trocar_topic, double dt) :
    _nh(nh),
    _ac(action_server),
    _timer(nh.createTimer(ros::Duration(dt), &HRCoMVSActionClient::_timerCB, this)),
    _dtwist_sub(nh.subscribe(dtwist_topic, 1, &HRCoMVSActionClient::_dTwistCB, this)),
    _p_trocar_sub(nh.subscribe(p_trocar_topic, 1, &HRCoMVSActionClient::_pTrocarCB, this)) {
       
        // Initialize task
        _task.values = std::vector<double>(6, 0.);
        _task.is_velocity = true;

        // Initialize trocar position
        _p_trocar.is_empty = true;

        _ac.waitForServer();
}


HRCoMVSActionClient::~HRCoMVSActionClient() {
    _nh.shutdown();
    _ac.cancelAllGoals();
    _dtwist_sub.shutdown();
    _p_trocar_sub.shutdown();
}
