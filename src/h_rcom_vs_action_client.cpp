#include <h_rcom_vs/h_rcom_vs_action_client.h>


// Callbacks
void HRCoMVSActionClient::_twistCB(const geometry_msgs::TwistConstPtr& twist_msg) {
    // Sets _task
    _task.values[0] = twist_msg->linear.x;
    _task.values[1] = twist_msg->linear.y;
    _task.values[2] = twist_msg->linear.z;
    _task.values[3] = twist_msg->angular.x;
    _task.values[4] = twist_msg->angular.y;
    _task.values[5] = twist_msg->angular.z;

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
HRCoMVSActionClient::HRCoMVSActionClient(ros::NodeHandle& nh, std::string& action_server, std::string& twist_topic, std::string& p_trocar_topic, double dt) :
    _nh(nh),
    _ac(action_server),
    _timer(nh.createTimer(ros::Duration(dt), &HRCoMVSActionClient::_timerCB, this)),
    _twist_sub(nh.subscribe(twist_topic, 1, &HRCoMVSActionClient::_twistCB, this)),
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
    _twist_sub.shutdown();
    _p_trocar_sub.shutdown();
}
