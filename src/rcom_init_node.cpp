#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Image.h>

#include <rcom_msgs/rcomAction.h>
#include <rcom_msgs/rcomGoal.h>


ros::Subscriber img_sub;
ros::Publisher img0_pub;
bool img0_init = false;
sensor_msgs::Image img0;

void img_cb(const sensor_msgs::ImageConstPtr& img) {
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

rcom_msgs::rcomGoal rnd_rcom_velocity_goal(int task_dim, double max_vel=1.) {

    rcom_msgs::rcomGoal rnd_goal;
    rnd_goal.states.p_trocar.is_empty = true;
    rnd_goal.states.task.is_velocity = true;

    std::stringstream ss;
    ss << "Generate random task: (";
    for (int i = 0; i < task_dim; i++) {
        rnd_goal.states.task.values.push_back(max_vel*rnd_uniform(gen));
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

    // Publish initial image
    img_sub = nh.subscribe("camera/image_raw", 1, img_cb);
    img0_pub = nh.advertise<sensor_msgs::Image>("visual_servo/img0", 1);

    // Execute random goal after initial image is being published
    while (!img0_init) {
        ros::Duration(0.1).sleep();
    }
    auto rnd_goal = rnd_rcom_velocity_goal(4, 0.5);
    ac.sendGoal(rnd_goal);

    ros::waitForShutdown();
    
    return 0;
}
