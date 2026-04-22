#ifndef MY_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_
#define MY_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>

namespace my_franka_controllers {

class TrajectoryGenerator : public rclcpp::Node {
public:
    TrajectoryGenerator();

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    bool trajectory_active{false};
    rclcpp::Time t_start;
    double duration{0.0};

    // Limits
    static constexpr double SAFETY_FACTOR = 0.2;
    const double max_linear_vel{2.00 * SAFETY_FACTOR};   // m/s, not sure what to place here
    const double max_angular_vel{2.62 * SAFETY_FACTOR};  // rad/s, same
    const double max_linear_acc{1.0 * SAFETY_FACTOR};  // m/s^2
    const double max_angular_acc{1.5 * SAFETY_FACTOR}; // rad/s^2
    const double min_duration{0.5};        // seconds (safety margin)

    Eigen::Vector3d p_start, p_goal;
    Eigen::Quaterniond q_start, q_goal;
};

} // namespace my_franka_controllers

#endif // MY_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_