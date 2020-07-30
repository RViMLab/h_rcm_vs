#include <ros/ros.h>

#include <h_rcom_vs/h_rcom_vs_action_client.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "h_rcom_vs_client_node");
    ros::NodeHandle nh;
    auto spinner = ros::AsyncSpinner(2);
    spinner.start();

    std::string action_server, dtwist_topic, p_trocar_topic;
    double dt;

    nh.getParam("action_server", action_server);
    dtwist_topic = "visual_servo/dtwist";
    p_trocar_topic = "rcom/p_trocar";
    nh.getParam("dt", dt);

    HRCoMVSActionClient h_rcom_vs_ac(
        nh,
        action_server, dtwist_topic, p_trocar_topic,
        dt
    );

    ros::waitForShutdown();
    
    return 0;
}
