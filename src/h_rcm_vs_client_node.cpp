#include <ros/ros.h>

#include <h_rcm_vs/h_rcm_vs_action_client.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "h_rcm_vs_client_node");
    ros::NodeHandle nh;
    auto spinner = ros::AsyncSpinner(2);
    spinner.start();

    std::string action_server, twist_topic, p_trocar_topic;
    double dt;

    nh.getParam("action_server", action_server);
    twist_topic = "visual_servo/twist";
    p_trocar_topic = "rcm/p_trocar";
    nh.getParam("dt", dt);

    HRCMVSActionClient h_rcm_vs_ac(
        nh,
        action_server, twist_topic, p_trocar_topic,
        dt
    );

    ros::waitForShutdown();
    
    return 0;
}
