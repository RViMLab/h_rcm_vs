#include <ros/ros.h>

#include <h_rcom_vs/h_rcom_vs_action_client.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "bundle_node");
    ros::NodeHandle nh;

    std::string action_server, dtwist_topic, p_trocar_topic;
    double dt;

    HRCoMVSActionClient(
        nh,
        action_server, dtwist_topic, p_trocar_topic,
        dt
    );

    ros::spin();
    
    return 0;
}
