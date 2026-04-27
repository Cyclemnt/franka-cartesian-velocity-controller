#include "my_franka_controllers/trajectory_generator_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace my_franka_controllers {

TrajectoryGenerator::TrajectoryGenerator() : Node("trajectory_generator") {
    // this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    cmd_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_hqp_cartesian_controller/target_pose", 10);
    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&TrajectoryGenerator::goal_callback, this, std::placeholders::_1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 100 Hz
    timer = this->create_wall_timer(1ms, std::bind(&TrajectoryGenerator::timer_callback, this)); // 100Hz enough?
    
    RCLCPP_INFO(this->get_logger(), "Trajectory Generator Ready.");
}

void TrajectoryGenerator::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (trajectory_active) {
        RCLCPP_WARN(this->get_logger(), "Preempting active trajectory.");
        // return;  // Ignore
    }
    try {
        auto transform = tf_buffer->lookupTransform("world", "fr3_link8", tf2::TimePointZero);
        
        p_start << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z;
        q_start = Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
        
        p_goal << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        q_goal = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        
        q_start.normalize();
        q_goal.normalize();

        // Shortest path for quaternion
        if (q_start.dot(q_goal) < 0.0) q_goal.coeffs() *= -1.0;

        // DYNAMIC DURATION CALCULATION
        double linear_distance = (p_goal - p_start).norm();
        double angular_distance = q_start.angularDistance(q_goal);

        // Quintic peak velocity is 1.875 * dist / T
        // Quintic peak acceleration is 5.77 * dist / T^2
        double t_vel = 1.875 * std::max(linear_distance / max_linear_vel, angular_distance / max_angular_vel);
        double t_accel = std::sqrt(5.77 * std::max(linear_distance / max_linear_acc, angular_distance / max_angular_acc));

        // Use the longest required time to ensure neither limit is broken, plus a minimum floor
        duration = std::max({t_vel, t_accel, min_duration});
        t_start = this->get_clock()->now();
        trajectory_active = true;

        RCLCPP_INFO(this->get_logger(), "Moving to new goal. Duration: %.2fs", duration);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
}

void TrajectoryGenerator::timer_callback() {
    if (!trajectory_active) return;

    double t = (this->get_clock()->now() - t_start).seconds();
    
    geometry_msgs::msg::PoseStamped cmd_msg;
    cmd_msg.header.stamp = this->get_clock()->now();
    cmd_msg.header.frame_id = "world";

    if (t >= duration) {
        cmd_msg.pose.position.x = p_goal.x();
        cmd_msg.pose.position.y = p_goal.y();
        cmd_msg.pose.position.z = p_goal.z();
        cmd_msg.pose.orientation.w = q_goal.w();
        cmd_msg.pose.orientation.x = q_goal.x();
        cmd_msg.pose.orientation.y = q_goal.y();
        cmd_msg.pose.orientation.z = q_goal.z();
        trajectory_active = false;
        RCLCPP_INFO(this->get_logger(), "Trajectory Complete.");
    } else {
        double tau = t / duration;
        double s = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        // double ds = (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / duration;

        Eigen::Vector3d p_current = p_start + s * (p_goal - p_start);
        Eigen::Quaterniond q_current = q_start.slerp(s, q_goal);

        cmd_msg.pose.position.x = p_current.x();
        cmd_msg.pose.position.y = p_current.y();
        cmd_msg.pose.position.z = p_current.z();
        cmd_msg.pose.orientation.w = q_current.w();
        cmd_msg.pose.orientation.x = q_current.x();
        cmd_msg.pose.orientation.y = q_current.y();
        cmd_msg.pose.orientation.z = q_current.z();
    }

    cmd_pub->publish(cmd_msg);
}

} // namespace my_franka_controllers

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_franka_controllers::TrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}