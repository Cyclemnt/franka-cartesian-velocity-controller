#include "my_franka_controllers/cartesian_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace my_franka_controllers {

// -------------------------------------------------------------------------
// on_init: Declare parameters and allocate non-realtime resources
// -------------------------------------------------------------------------
controller_interface::CallbackReturn CartesianVelocityController::on_init() {
    auto node = get_node();

    // Declare ROS 2 parameters
    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names);
    node->declare_parameter("end_effector_frame", "fr3_link8");
    // node->declare_parameter("robot_description", "");

    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_configure: Read parameters, setup Pinocchio, allocate matrix sizes
// -------------------------------------------------------------------------
controller_interface::CallbackReturn CartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();

    // Retrieve Parameters
    joint_names = node->get_parameter("joint_names").as_string_array();
    end_effector_frame = node->get_parameter("end_effector_frame").as_string();
    std::string urdf_string = node->get_parameter("robot_description").as_string();

    // Build Pinocchio model from URDF string
    pinocchio::urdf::buildModelFromXML(urdf_string, model);
    data = std::make_shared<pinocchio::Data>(model);
    data_tmp = std::make_shared<pinocchio::Data>(model);
    ee_frame_id = model.getFrameId(end_effector_frame);

    // Set Eigen matrices
    size_t num_joints = joint_names.size();
    dq_max << 2.62, 2.62, 2.62, 2.62, 5.26, 5.26, 5.26; // rad/s
    dq_cmd.setZero();
    dq_cmd_prev.setZero();
    // Control law
    K_gain.setZero();
    K_gain.diagonal() << 10.0, 10.0, 10.0, 5.0, 5.0, 5.0; // P-Gains for [x, y, z, roll, pitch, yaw]
    jacobian.setZero();
    I6.setIdentity();
    JJt.setZero();
    JJt_inv.setZero();
    llt_solver.compute(Eigen::Matrix<double, 6, 6>::Identity());
    // Nullspace control
    K_null = 0.8;
    I7.setIdentity();
    grad_w_cached.setZero();
    // Franka mid configuration
    for (size_t i = 0; i < num_joints; ++i) {
        double upper = model.upperPositionLimit[i];
        double lower = model.lowerPositionLimit[i];
        // Check if limits are actually defined
        if (upper > 1e3 || lower < -1e3) q_mid(i) = 0.0; // For continuous joints
        else q_mid(i) = (upper + lower) / 2.0; // Else middle
    }

    // Initialize Realtime Buffer with an empty target
    TargetPose init_target;
    init_target.valid = false;
    rt_target_pose_ptr.initRT(init_target);

    // Setup target pose subscriber
    target_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        TargetPose target;

        target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        target.orientation.normalize();

        target.valid = true;

        rt_target_pose_ptr.writeFromNonRT(target);
    });

    // Setup error publisher
    error_pub = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", rclcpp::SystemDefaultsQoS());
    rt_error_pub = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(error_pub);

    RCLCPP_INFO(node->get_logger(), "CartesianVelocityController configured successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// command_interface_configuration: command velocity
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

// -------------------------------------------------------------------------
// state_interface_configuration: read position
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

// -------------------------------------------------------------------------
// on_activate: when controller switches from INACTIVE to ACTIVE
// -------------------------------------------------------------------------
controller_interface::CallbackReturn CartesianVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)  {
    // Read initial joint positions
    for (size_t i = 0; i < joint_names.size(); ++i) {
        q_current(i) = state_interfaces_[i].get_value();
    }

    // Where is the robot:
    pinocchio::forwardKinematics(model, *data, q_current);
    pinocchio::updateFramePlacements(model, *data);
    x_current = data->oMf[ee_frame_id].translation();
    quat_current = Eigen::Quaterniond(data->oMf[ee_frame_id].rotation());

    // Set the target to the current position to prevent jerking on startup
    x_target = x_current;
    quat_target = quat_current;

    // Clear any old targets in the buffer
    TargetPose reset;
    reset.valid = false;
    rt_target_pose_ptr.writeFromNonRT(reset);
    last_target_time = get_node()->now();

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated. Holding current pose.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_deactivate: send zero commands before shutting down
// -------------------------------------------------------------------------
controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)  {
    for (size_t i = 0; i < joint_names.size(); ++i) {
        command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// update: 1kHz real-time loop. NO MEMORY ALLOCATION
// -------------------------------------------------------------------------
controller_interface::return_type CartesianVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& period)  {
    const double dt = period.seconds();

    // =========================================================
    // READ TARGET + WATCHDOG TIMEOUT
    // =========================================================
    // Read target from subscriber
    TargetPose* target_ptr = rt_target_pose_ptr.readFromRT();
    if (target_ptr && target_ptr->valid) {
        x_target = target_ptr->position;
        quat_target = target_ptr->orientation;
        last_target_time = time;
    }
    
    // Safe stop if no target for 500ms
    if ((time - last_target_time).seconds() > 0.5) {
        for (size_t i = 0; i < 7; ++i) command_interfaces_[i].set_value(0.0);
        dq_cmd_prev.setZero();
        return controller_interface::return_type::OK;
    }

    // =========================================================
    // READ JOINTS + PINOCCHIO KINEMATICS
    // =========================================================
    // Read Current Joint States
    for (size_t i = 0; i < joint_names.size(); ++i) {
        q_current(i) = state_interfaces_[i].get_value();
    }

    // Pinocchio Kinematics
    pinocchio::computeJointJacobians(model, *data, q_current);
    pinocchio::updateFramePlacements(model, *data);
    pinocchio::getFrameJacobian(model, *data, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

    // Update vectors
    x_current = data->oMf[ee_frame_id].translation();
    quat_current = Eigen::Quaterniond(data->oMf[ee_frame_id].rotation());
    quat_current.normalize();

    // =========================================================
    // CARTESIAN ERROR
    // =========================================================
    // Compute errors
    pos_error = x_target - x_current;  // Position error

    quat_error = quat_target * quat_current.inverse();   // Quaternion error
    if (quat_error.w() < 0) quat_error.coeffs() *= -1.0; // shortest path for rotation

    ori_error = 2.0 * quat_error.vec();   // Orientation error approximation but faster than pinocchio::log3(quat_error.toRotationMatrix())

    twist_error << pos_error, ori_error;  // Total twist error vector
    const double error_norm = twist_error.norm();

    // =========================================================
    // ADAPTATIVE DAMPED PSEUDO-INVERSE
    // =========================================================
    // J^# = J^T * (J * J^T + lambda^2 * I)^-1
    JJt.noalias() = jacobian * jacobian.transpose();

    // Adaptive lambda
    const double manipulability_proxy = JJt.trace() / 6.0;  // manipulability proxy O(n), avoiding déterminant
    {
        constexpr double lambda_min = 1e-4;
        constexpr double lambda_max = 5e-2;
        constexpr double w_threshold = 1e-3;  // singularity warning region
        lambda_damping = lambda_min;
        if (manipulability_proxy < w_threshold) {
            const double r = 1.0 - (manipulability_proxy / w_threshold);
            lambda_damping = lambda_min + (lambda_max - lambda_min) * r * r;
        }
    }

    // Computing J_pinv
    JJt.diagonal().array() += lambda_damping * lambda_damping;
    llt_solver.compute(JJt);
    JJt_inv.noalias() = llt_solver.solve(I6);
    J_pinv.noalias() = jacobian.transpose() * JJt_inv;
    
    // =========================================================
    // MAIN TASK with fade-in
    // =========================================================
    // Avoid speed bumps if target is far at activation
    activation_ramp = std::min(activation_ramp + dt / 0.5, 1.0); // 500 ms ramp

    dq_task.noalias() = J_pinv * (K_gain * twist_error);

    // =========================================================
    // NULLSPACE manipulability gradient
    // =========================================================
    // manipulability gradient updated at 50 Hz
    grad_cycle_counter++;
    if (grad_cycle_counter >= GRAD_CYCLE_PERIOD) {
        grad_cycle_counter = 0;

        const double w_curr = std::sqrt((jacobian * jacobian.transpose()).determinant() + 1e-12);

        constexpr double delta = 1e-4;
        for (int i = 0; i < 7; ++i) {
            q_perturbed = q_current;
            q_perturbed(i) += delta;

            pinocchio::computeJointJacobians(model, *data_tmp, q_perturbed);
            pinocchio::updateFramePlacements(model, *data_tmp);
            J_tmp.setZero();
            pinocchio::getFrameJacobian(model, *data_tmp, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_tmp);


            Eigen::Matrix<double, 6, 6> JJt_tmp = J_tmp * J_tmp.transpose();
            JJt_tmp.diagonal().array() += 1e-12;
            const double w_tmp = std::sqrt(JJt_tmp.determinant());
            grad_w_cached(i) = (w_tmp - w_curr) / delta;
        }

        if (grad_w_cached.norm() > 1e-8) grad_w_cached /= grad_w_cached.norm();
    }

    // Repulse joint limits
    for (int i = 0; i < 7; ++i) {
        constexpr double eps = 1e-3;
        const double dist_upper = std::max(model.upperPositionLimit[i] - q_current(i), eps);
        const double dist_lower = std::max(q_current(i) - model.lowerPositionLimit[i], eps);
        const double range  = model.upperPositionLimit[i] - model.lowerPositionLimit[i];
        dq_limit(i) = (dist_upper < 0.3 * range || dist_lower < 0.3 * range) ? (1.0 / dist_lower - 1.0 / dist_upper) : 0.0;
    }

    // Fusion + project into nullspace
    // Blend nullspace when goal is reached toavoid vibrations
    const double null_blend = (error_norm > 0.01) ? 1.0 : (error_norm < 0.001) ? 0.0 : (error_norm - 0.001) / 0.009;

    dq_null.noalias() = K_null * (w_manip_weight * grad_w_cached + w_limit_weight * dq_limit);
    N.noalias() = I7 - J_pinv * jacobian;

    // Total Joint Velocity Command
    dq_cmd = activation_ramp * dq_task + null_blend * N * dq_null;

    // =========================================================
    // SECURITIES
    // =========================================================
    // Global scaling
    {
        double scale = 1.0;
        for (int i = 0; i < 7; ++i) {
            const double ratio = std::abs(dq_cmd(i)) / dq_max(i);
            if (ratio > 1.0) scale = std::min(scale, 1.0 / ratio);
        }
        dq_cmd *= scale;
    }

    // Joint clamping regarding joint limit proximity
    {
        constexpr double margin      = 0.262; // rad (slow down zone)
        constexpr double hard_margin = 0.020; // rad (stop zone)
        for (int i = 0; i < 7; ++i) {
            const double dist_upper = model.upperPositionLimit[i] - q_current(i);
            const double dist_lower = q_current(i) - model.lowerPositionLimit[i];

            if (dq_cmd(i) > 0.0 && dist_upper < margin) {
                dq_cmd(i) *= (dist_upper < hard_margin) ? 0.0 : (dist_upper - hard_margin) / (margin - hard_margin);
            }
            if (dq_cmd(i) < 0.0 && dist_lower < margin) {
                dq_cmd(i) *= (dist_lower < hard_margin) ? 0.0 : (dist_lower - hard_margin) / (margin - hard_margin);
            }
        }
    }

    // =========================================================
    // LOW PASS FILTER + WRITE IN HARDWARE INTERFACE
    // =========================================================
    // Adaptative alpha
    const double alpha = (error_norm > 0.05) ? 0.4 : 0.15;
    dq_cmd = alpha * dq_cmd + (1.0 - alpha) * dq_cmd_prev;
    dq_cmd_prev = dq_cmd;

    // Write Filtered Commands to Hardware Interface
    for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(dq_cmd(i));
    }
    
    // =========================================================
    // DIAGNOSTICS PUBLISHING
    // =========================================================
    // try_lock() to not block the 1000Hz thread
    if (rt_error_pub && rt_error_pub->trylock()) {
        rt_error_pub->msg_.header.stamp    = time;
        rt_error_pub->msg_.header.frame_id = end_effector_frame;
        rt_error_pub->msg_.twist.linear.x  = pos_error.x();
        rt_error_pub->msg_.twist.linear.y  = pos_error.y();
        rt_error_pub->msg_.twist.linear.z  = pos_error.z();
        rt_error_pub->msg_.twist.angular.x = ori_error.x();
        rt_error_pub->msg_.twist.angular.y = ori_error.y();
        rt_error_pub->msg_.twist.angular.z = ori_error.z();
        rt_error_pub->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

}  // namespace my_franka_controllers

// Export the class to ROS 2 Pluginlib
PLUGINLIB_EXPORT_CLASS(my_franka_controllers::CartesianVelocityController, controller_interface::ControllerInterface)