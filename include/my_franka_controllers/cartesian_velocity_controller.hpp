#ifndef MY_FRANKA_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define MY_FRANKA_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// ROS 2 Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// Eigen & Pinocchio
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace my_franka_controllers {

class CartesianVelocityController : public controller_interface::ControllerInterface {
public:
    CartesianVelocityController() = default;
    ~CartesianVelocityController() = default;

    // -------------------------------------------------------------------------
    // controller_interface::ControllerInterface Overrides
    // -------------------------------------------------------------------------
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    // Custom structure to avoid shared_ptr<PoseStamped>
    struct TargetPose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool valid{false};
    };

    // ROS 2 Parameters & Communication
    std::vector<std::string> joint_names;
    std::string end_effector_frame;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
    realtime_tools::RealtimeBuffer<TargetPose> rt_target_pose_ptr;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> error_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> rt_error_pub;

    // Pinocchio Kinematics
    pinocchio::Model model;
    std::shared_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex ee_frame_id;

    // Pre-allocated Math Variables
    Eigen::Matrix<double, 7, 1> q_current;
    Eigen::Matrix<double, 7, 1> dq_cmd;
    Eigen::Matrix<double, 7, 1> dq_max;
    
    Eigen::Vector3d x_current;
    Eigen::Quaterniond quat_current;
    
    Eigen::Vector3d x_target;
    Eigen::Quaterniond quat_target;

    Eigen::Vector3d pos_error;
    Eigen::Quaterniond quat_error;
    Eigen::Vector3d ori_error;

    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 6, 1> twist_error;
    Eigen::Matrix<double, 6, 6> K_gain;

    // Nullspace control variables
    double K_null;
    Eigen::Matrix<double, 7, 1> q_mid;
    Eigen::Matrix<double, 7, 1> dq_null;
    Eigen::Matrix<double, 7, 7> I7;
    Eigen::Matrix<double, 7, 7> N;
    
    // Damped pseudo-inverse variables
    Eigen::Matrix<double, 6, 6> I6;
    Eigen::Matrix<double, 6, 6> JJt;
    Eigen::Matrix<double, 6, 6> JJt_inv;
    Eigen::Matrix<double, 7, 6> J_pinv;
    double lambda_damping;
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt_solver;

    // Low pass filter
    Eigen::Matrix<double, 7, 1> dq_cmd_prev;
    double filter_alpha;
};

}  // namespace my_franka_controllers

#endif  // MY_FRANKA_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_