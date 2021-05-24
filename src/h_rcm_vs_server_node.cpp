#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <h_rcm_vs/h_rcm_vs_action_server.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "rcm_node");
    auto nh = ros::NodeHandle();
    auto spinner = ros::AsyncSpinner(2);
    spinner.start();

    // Parameters
    std::string action_server, control_client;
    std::vector<double> kpt, kit, kdt, kprcm, kircm, kdrcm;
    double lambda0, dt;
    std::string planning_group;
    double alpha;
    std::string link_pi, link_pip1;
    double t1_td, t1_p_trocar, t2_td, t2_p_trocar;
    std::vector<double> t_td_scale;
    int max_iter;
    double exp_smooth, dumping;
    bool rcm_priority;

    nh.getParam("action_server", action_server);
    nh.getParam("control_client", control_client);
    nh.getParam("kpt", kpt);
    nh.getParam("kit", kit);
    nh.getParam("kdt", kdt);
    nh.getParam("kprcm", kprcm);
    nh.getParam("kircm", kircm);
    nh.getParam("kdrcm", kdrcm);
    nh.getParam("lambda0", lambda0);
    nh.getParam("dt", dt);
    nh.getParam("planning_group", planning_group);
    nh.getParam("alpha", alpha);
    nh.getParam("link_pi", link_pi);
    nh.getParam("link_pip1", link_pip1);
    nh.getParam("t1_td", t1_td);
    nh.getParam("t_td_scale", t_td_scale);
    nh.getParam("t1_p_trocar", t1_p_trocar);
    nh.getParam("t2_td", t2_td);
    nh.getParam("t2_p_trocar", t2_p_trocar);
    nh.getParam("max_iter", max_iter);
    nh.getParam("exp_smooth", exp_smooth);
    nh.getParam("dumping", dumping);
    nh.getParam("rcm_priority", rcm_priority);

    // Initialize position
    auto move_group = moveit::planning_interface::MoveGroupInterface(planning_group);
    move_group.setMaxVelocityScalingFactor(0.01);

    // // Go home
    // move_group.setNamedTarget("home");
    // move_group.move();
    // move_group.stop();

    // Set an initial pose, corresponding to p_trocar
    auto joint_values = move_group.getCurrentJointValues();

    // initial state
    // joint_values[0] =  0.79396429;
    // joint_values[1] =  1.76694670;
    // joint_values[2] = -1.56914485;
    // joint_values[3] = -1.04951022;
    // joint_values[4] =  0.76358239;
    // joint_values[5] = -0.66000193;
    // joint_values[6] =  1.67966506;
    joint_values[0] =  0.26475579;
    joint_values[1] =  0.87986609;
    joint_values[2] = -0.04439981;
    joint_values[3] = -1.17369424;
    joint_values[4] =  0.43113507;
    joint_values[5] = -0.60086285;
    joint_values[6] =  0.47310088;
                        
// -2.0963101016467607, -0.6216701022126265, 2.5967572522689633, -1.529268530677615, 0.42836403917667276, -0.4396927029633622, 0.6832715948093737
//best: 0.34244437304967695, 0.8488681164795888, -0.07750575751807416, -1.110980971739215, 0.605830851678601, -0.5359390918224382, 0.31923991761162096
//best re pos: position: 0.26475579621944717, 0.8798660931646686, -0.04439981557750074, -1.1736942408148117, 0.43113507958050823, -0.600862851266197, 0.473100887061162

    move_group.setJointValueTarget(joint_values);
    move_group.move();
    move_group.stop();

    move_group.setMaxVelocityScalingFactor(1.0);

    // Action server
    rcm::HRCMVSActionServer rcm_as(
        nh, action_server, control_client,
        kpt, kit, kdt, kprcm, kircm, kdrcm, lambda0, dt,
        planning_group, alpha, link_pi, link_pip1,
        t1_td, t1_p_trocar, t2_td, t2_p_trocar, t_td_scale, max_iter,
        exp_smooth, dumping, rcm_priority
    );

    ros::waitForShutdown();

    return 0;
}
