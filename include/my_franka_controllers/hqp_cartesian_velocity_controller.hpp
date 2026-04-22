#ifndef MY_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define MY_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// ROS 2 Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// Eigen & HQP Architecture
#include <Eigen/Dense>
#include <task/task.h>
#include <qp/QPSolver.h>
#include <robot_kinematics/FrankaKinematics.hpp>

namespace my_franka_controllers {

class HqpCartesianVelocityController : public controller_interface::ControllerInterface {
public:
    HqpCartesianVelocityController() = default;
    ~HqpCartesianVelocityController() = default;

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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
    realtime_tools::RealtimeBuffer<TargetPose> rt_target_pose_ptr;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> error_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> rt_error_pub;

    // HQP Components
    std::shared_ptr<FrankaKinematics> kinematics;
    std::shared_ptr<QPSolver> solver;
    
    // The Task Stack: Holds pointers to the base Task class
    std::vector<std::shared_ptr<Task>> task_stack;
    std::shared_ptr<Pose> pose_task;

    // Math Variables
    Eigen::Matrix<double, 7, 1> q_current;
    Eigen::Matrix<double, 7, 1> dq_cmd;
    Eigen::Matrix<double, 7, 1> dq_cmd_prev;
    Eigen::Matrix<double, 7, 1> dq_max;
    
    Eigen::Vector3d x_target;
    Eigen::Quaterniond quat_target;

    // Security
    rclcpp::Time last_target_time{0};
    double activation_ramp{0.0};
};

}  // namespace my_franka_controllers

#endif // MY_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_