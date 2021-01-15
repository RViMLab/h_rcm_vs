#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Image.h>

#include <rcom_msgs/rcom.h>
#include <rcom_msgs/rcomAction.h>
#include <rcom_msgs/rcomGoal.h>


ros::Subscriber state_sub;
ros::Subscriber img_sub;
ros::Publisher img0_pub;
bool rcom0_init = false;
rcom_msgs::rcom rcom0;
bool img0_init = false;
sensor_msgs::Image img0;

void stateCB(const rcom_msgs::rcomConstPtr& rcom) {
    // Initialize rcom, after action server started
    if (!rcom0_init) {
        rcom0 = *rcom;
        rcom0_init = true;
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

rcom_msgs::rcomGoal rcomGoalFromVel(std::vector<double> vel, std::vector<double> apd) {
    rcom_msgs::rcomGoal goal;
    goal.states.p_trocar.is_empty = true;
    goal.states.task.is_velocity = true;

    goal.states.task.values = vel;

    for (int i = 0; i < apd.size(); i++) {
        goal.states.task.values.push_back(apd[i]);
    }

    return goal;
}

rcom_msgs::rcomGoal rndRCoMPositionGoal(int task_dim, double max_pos=1.) {

    rcom_msgs::rcomGoal rnd_goal;
    rnd_goal.states.p_trocar.is_empty = true;
    rnd_goal.states.task.is_velocity = false;

    std::stringstream ss;
    ss << "Generate random task: (";
    for (int i = 0; i < task_dim; i++) {
        rnd_goal.states.task.values.push_back(rcom0.task.values[i] + max_pos*rnd_uniform(gen));
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

// Node to randomly initialize rcom for demonstration purposes
int main(int argc, char** argv) {
    ros::init(argc, argv, "rcom_init_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::string action_server;
    nh.getParam("action_server", action_server);

    actionlib::SimpleActionClient<rcom_msgs::rcomAction> ac(action_server);
    ac.waitForServer();

    ROS_INFO("rcom_init_node: Action server %s running, starting node", action_server.c_str());

    ros::Duration(1.0).sleep();  // wait for robot to settle

    // Fetch initial position
    state_sub = nh.subscribe("h_rcom_vs/RCoM_ActionServer/state", 1, stateCB);

    // Publish initial image
    img_sub = nh.subscribe("camera/image_raw", 1, imgCB);
    img0_pub = nh.advertise<sensor_msgs::Image>("visual_servo/img0", 1);

    // Execute random goal after initial image is being published
    while (!img0_init) {
        ros::Duration(0.1).sleep();
    }
    
    // // Send random position goal
    // auto rnd_goal = rndRCoMPositionGoal(4, 0.02);
    // ac.sendGoal(rnd_goal);

    // Send velocity goal at rate
    auto rate = ros::Rate(25);

    // auto rnd_vel = rndVelocity(4, 1.);
    std::vector<double> rnd_vel = {1.0, 0.5, 0.5, 0.0};
    auto goal = rcomGoalFromVel(rnd_vel, std::vector<double>(2, 0.));

    int counter = 0;
    while (counter < 200) {
        counter++;
        ac.sendGoal(goal);
        rate.sleep();
    }

    // Run to ensure p_trocar convergence
    std::vector<double> zero_vel(rnd_vel.size(), 0.);
    goal = rcomGoalFromVel(zero_vel, std::vector<double>(2, 0.));

    counter = 0;
    while (counter < 100) {
        counter++;
        ac.sendGoal(goal);
        rate.sleep();
    }



    ros::waitForShutdown();
    
    return 0;
}
