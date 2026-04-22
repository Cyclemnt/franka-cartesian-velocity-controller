#include "my_franka_controllers/hqp_cartesian_velocity_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace my_franka_controllers {

// -------------------------------------------------------------------------
// on_init
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_init() {
    auto node = get_node();
    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names);

    // DH parameters
    auto declare_if_not_exists = [&](const std::string & name, const auto & default_val) {
        if (!node->has_parameter(name)) {
            node->declare_parameter(name, default_val);
        }
    };
    declare_if_not_exists("mod_DH.a", std::vector<double>());
    declare_if_not_exists("mod_DH.alpha", std::vector<double>());
    declare_if_not_exists("mod_DH.d", std::vector<double>());
    declare_if_not_exists("mod_DH.theta", std::vector<double>());
    declare_if_not_exists("A7e", std::vector<double>());

    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_configure
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();
    dq_max << 2.62, 2.62, 2.62, 2.62, 5.26, 5.26, 5.26;
    dq_cmd.setZero();
    dq_cmd_prev.setZero();

    // Load DH Parameters
    auto a = node->get_parameter("mod_DH.a").as_double_array();
    auto alpha = node->get_parameter("mod_DH.alpha").as_double_array();
    auto d = node->get_parameter("mod_DH.d").as_double_array();
    auto theta = node->get_parameter("mod_DH.theta").as_double_array();
    auto a7e = node->get_parameter("A7e").as_double_array();

    std::vector<double> mDH;
    mDH.insert(mDH.end(), a.begin(), a.end());
    mDH.insert(mDH.end(), alpha.begin(), alpha.end());
    mDH.insert(mDH.end(), d.begin(), d.end());
    mDH.insert(mDH.end(), theta.begin(), theta.end());

    // Initialize Kinematics
    kinematics = std::make_shared<FrankaKinematics>();
    if (mDH.size() == 28 && a7e.size() == 16) {
        kinematics->setParameters(mDH, a7e);
    } else {
        RCLCPP_ERROR(node->get_logger(), "DH Parameter mismatch");
        return controller_interface::CallbackReturn::ERROR;
    }

    kinematics->getSelectDOF()->assign(7, true);
    kinematics->getSelectTask()->assign(6, true);

    // Initialize Solver and Task Stack
    solver = std::make_shared<QPSolver>();
    task_stack.clear();

    // Create Primary Pose Task
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(6); 
    pose_task = std::make_shared<Pose>(kinematics.get(), '=', weights);
    pose_task->setGain(10.0);
    
    // Add to the stack (push_back new tasks here)
    task_stack.push_back(pose_task);

    // Target Subscription
    target_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            TargetPose target;
            target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
            target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).normalized();
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
// command_interface_configuration
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration HqpCartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

// -------------------------------------------------------------------------
// state_interface_configuration
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration HqpCartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

// -------------------------------------------------------------------------
// on_activate
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_activate(const rclcpp_lifecycle::State&) {
    for (size_t i = 0; i < 7; ++i) { q_current(i) = state_interfaces_[i].get_value(); }
    kinematics->updateJointStates(q_current);
    
    // If FrankaKinematics allows reading current pose, initialize x_target here to avoid jumps.
    // x_target = ...
    // quat_target = ...

    activation_ramp = 0.0;
    last_target_time = get_node()->now();

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_deactivate
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State&) {
    for (size_t i = 0; i < 7; ++i) { command_interfaces_[i].set_value(0.0); }
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// update: The 1000Hz Loop
// -------------------------------------------------------------------------
controller_interface::return_type HqpCartesianVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
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
    // READ JOINTS + UPDATE KINEMATICS
    // =========================================================
    // Read Current Joint States
    for (size_t i = 0; i < 7; ++i) { q_current(i) = state_interfaces_[i].get_value(); }
    kinematics->updateJointStates(q_current);
    
    // Pass the goal to the kinematics object so the Pose task can calculate 'b'
    kinematics->setDesiredPose(x_target, quat_target);

    // =========================================================
    // HQP SOLVER
    // =========================================================
    // Formulate and Solve QP using the Task Stack
    try {
        solver->clear(); // Reset Gurobi model constraints
        solver->addVariables(7, 'C'); // 7 continuous joint velocities
        
        // Loop through all tasks and add them to the solver
        for(const auto& task : task_stack) {
            // Assuming tasks are all Equality '=' constraints for now
            // If Task class stores inequality type, pass it here
            solver->addConstraints(task->get_A(), task->get_b(), '=', false);
        }
        
        // Objective: Minimize kinetic energy/joint velocities (1/2 * dq^T * I * dq)
        solver->setObjectiveFunction(Eigen::MatrixXd::Identity(7, 7)); 

        // Solve
        solver->solve();
        dq_cmd = solver->getVariablesValue();

    } catch (const std::exception& e) {
        // Gurobi Exception Handling (e.g., Infeasible model, license error)
        for (size_t i = 0; i < 7; ++i) command_interfaces_[i].set_value(0.0);
        return controller_interface::return_type::OK;
    }

    // =========================================================
    // SAFETY
    // =========================================================
    // Safety Constraints & Scaling
    activation_ramp = std::min(activation_ramp + dt / 0.5, 1.0);
    dq_cmd *= activation_ramp;

    // Scaling
    {
        double scale = 1.0;
        for (int i = 0; i < 7; ++i) {
            const double ratio = std::abs(dq_cmd(i)) / dq_max(i);
            if (ratio > 1.0) scale = std::min(scale, 1.0 / ratio);
        }
        dq_cmd *= scale;
    }

    // Low Pass Filter
    const double alpha = 0.15; // Smooths out harsh Gurobi step responses
    dq_cmd = alpha * dq_cmd + (1.0 - alpha) * dq_cmd_prev;
    dq_cmd_prev = dq_cmd;

    // =========================================================
    // WRITE IN HARDWARE INTERFACE
    // =========================================================
    for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(dq_cmd(i));
    }

    // =========================================================
    // DIAGNOSTICS PUBLISHING
    // =========================================================
    // try_lock() to not block the 1000Hz thread
    Eigen::VectorXd error = kinematics->getError();
    Eigen::Vector3d pos_error = error.head(3);
    Eigen::Vector3d ori_error = error.tail(3);
    if (rt_error_pub && rt_error_pub->trylock()) {
        rt_error_pub->msg_.header.stamp    = time;
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

} // namespace my_franka_controllers

PLUGINLIB_EXPORT_CLASS(my_franka_controllers::HqpCartesianVelocityController, controller_interface::ControllerInterface)