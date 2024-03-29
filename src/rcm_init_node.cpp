#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Image.h>

#include <rcm_msgs/rcm.h>
#include <rcm_msgs/rcmAction.h>
#include <rcm_msgs/rcmGoal.h>


ros::Subscriber state_sub;
ros::Subscriber img_sub;
ros::Publisher img0_pub;
bool rcm0_init = false;
rcm_msgs::rcm rcm0;
bool img0_init = false;
sensor_msgs::Image img0;

void stateCB(const rcm_msgs::rcmConstPtr& rcm) {
    // Initialize rcm, after action server started
    if (!rcm0_init) {
        rcm0 = *rcm;
        rcm0_init = true;
        ROS_INFO("Initialized remote center of motion.");
    }
}

void imgCB(const sensor_msgs::ImageConstPtr& img) {
    // Initialize image, after action server started
    if (!img0_init) {
        img0 = *img; 
        img0_init = true;
        ROS_INFO("Initialized image.");
    }
    else {
        img0_pub.publish(img0);
    }
}

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> rnd_uniform(-1., 1.);

std::vector<double> rndVelocity(int task_dim, double max_vel=1.) {
    std::vector<double> vel;

    for (int i = 0; i < task_dim; i++) {
        vel.push_back(max_vel*rnd_uniform(gen));
    }

    return vel;
}

rcm_msgs::rcmGoal rcmGoalFromVel(std::vector<double> vel) {
    rcm_msgs::rcmGoal goal;
    goal.states.p_trocar.is_empty = true;
    goal.states.task.is_velocity = true;

    goal.states.task.values = vel;

    return goal;
}

rcm_msgs::rcmGoal rndRCMPositionGoal(int task_dim, double max_pos=1.) {

    rcm_msgs::rcmGoal rnd_goal;
    rnd_goal.states.p_trocar.is_empty = true;
    rnd_goal.states.task.is_velocity = false;

    std::stringstream ss;
    ss << "Generate random task: (";
    for (int i = 0; i < task_dim; i++) {
        rnd_goal.states.task.values.push_back(rcm0.task.values[i] + max_pos*rnd_uniform(gen));
        ss << rnd_goal.states.task.values[i];
        if (i <= task_dim - 2) {
            ss << ", ";
        }
        else {
            ss << ")";
        }   
    }
    ROS_INFO("%s", ss.str().c_str());

    return rnd_goal;
}

// Node to randomly initialize rcm for demonstration purposes
int main(int argc, char** argv) {
    ros::init(argc, argv, "rcm_init_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::string action_server;
    nh.getParam("action_server", action_server);

    actionlib::SimpleActionClient<rcm_msgs::rcmAction> ac(action_server);
    ac.waitForServer();

    ROS_INFO("rcm_init_node: Action server %s running, starting node", action_server.c_str());

    ros::Duration(1.0).sleep();  // wait for robot to settle

    // Fetch initial position
    state_sub = nh.subscribe("h_rcm_vs/RCM_ActionServer/state", 1, stateCB);

    // Publish initial image
    img_sub = nh.subscribe("camera/image_raw", 1, imgCB);
    img0_pub = nh.advertise<sensor_msgs::Image>("visual_servo/img0", 1);

    // Execute random goal after initial image is being published
    while (!img0_init) {
        ros::Duration(0.1).sleep();
    }
    
    // // Send random position goal
    // auto rnd_goal = rndRCMPositionGoal(4, 0.02);
    // ac.sendGoal(rnd_goal);

    // Send velocity goal at rate
    auto rate = ros::Rate(25);

    auto rnd_vel = rndVelocity(6, 0.05);
    // std::vector<double> rnd_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5};  // x,y,z,r,p,y wrt camera frame
    auto goal = rcmGoalFromVel(rnd_vel);

    int counter = 0;
    while (counter < 200) {
        counter++;
        ac.sendGoal(goal);
        rate.sleep();
    }

    // Run to ensure p_trocar convergence
    std::vector<double> zero_vel(rnd_vel.size(), 0.);
    goal = rcmGoalFromVel(zero_vel);

    counter = 0;
    while (counter < 100) {
        counter++;
        ac.sendGoal(goal);
        rate.sleep();
    }



    ros::waitForShutdown();
    
    return 0;
}
