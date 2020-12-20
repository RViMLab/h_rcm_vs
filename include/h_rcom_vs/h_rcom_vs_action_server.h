#pragma once

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <rcom_impl/base_rcom_action_server.h>


namespace rcom
{

class HRCoMVSActionServer : BaseRCoMActionServer {

    public:

        HRCoMVSActionServer(      
            ros::NodeHandle nh, std::string action_server, std::string control_client, 
            std::vector<double> kt, std::vector<double> krcm, double lambda0, double dt, 
            std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
            double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, std::vector<double> t_td_scale, int max_iter
        );

    private:

        // Compute task jacobian
        virtual Eigen::MatrixXd _computeTaskJacobian(moveit::core::RobotStatePtr robot_state) override;

        // Compute error between current and desired values via forward kinematics
        virtual Eigen::VectorXd _computeTaskForwardKinematics(std::vector<double>& q) override;

        // Transform the task from camera to world frame
        virtual Eigen::VectorXd _transformTask(Eigen::VectorXd& td) override;

};


HRCoMVSActionServer::HRCoMVSActionServer(
    ros::NodeHandle nh, std::string action_server, std::string control_client, 
    std::vector<double> kt, std::vector<double> krcm, double lambda0, double dt, 
    std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
    double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, std::vector<double> t_td_scale, int max_iter
) : BaseRCoMActionServer(
    nh, action_server, control_client, 
    kt, krcm, lambda0, dt, 
    planning_group, alpha, link_pi, link_pip1, 
    t1_td, t1_p_trocar, t2_td, t2_p_trocar, t_td_scale, max_iter) {   };


Eigen::MatrixXd HRCoMVSActionServer::_computeTaskJacobian(moveit::core::RobotStatePtr robot_state) {

    // Compute task jacobian
    Eigen::MatrixXd Jt;

    robot_state->getJacobian(
        robot_state->getJointModelGroup(_planning_group),
        robot_state->getLinkModel(_link_pip1),
        Eigen::Vector3d::Zero(),
        Jt
    );

    return Jt.topRows(4);
};


Eigen::VectorXd HRCoMVSActionServer::_computeTaskForwardKinematics(std::vector<double>& q) {

    // Compute forward kinematics
    auto robot_model = _move_group.getRobotModel();
    auto robot_state = moveit::core::RobotState(robot_model);

    for (int i = 0; i < q.size(); i++) {
        robot_state.setJointPositions(_move_group.getJointNames()[i], &q[i]);
    }

    tf::Matrix3x3 rot;
    tf::matrixEigenToTF(robot_state.getGlobalLinkTransform(_link_pip1).rotation(), rot);
    double r, p, y;
    rot.getRPY(r, p, y);

    Eigen::VectorXd t(4); 
    t << robot_state.getGlobalLinkTransform(_link_pip1).translation(), r;

    return t;
};


Eigen::VectorXd HRCoMVSActionServer::_transformTask(Eigen::VectorXd& td) {

    auto robot_state = _move_group.getCurrentState();

    // Rotate task from camera frame to world frame
    Eigen::MatrixXd R(6, 6);
    R << robot_state->getGlobalLinkTransform(_link_pip1).rotation(), Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), robot_state->getGlobalLinkTransform(_link_pip1).rotation();

    td = R*td;
    return td.topRows(4);
}

} // namespace rcom
