# My Franka ROS 2 Controllers

This package implements a high-performance Cartesian Velocity Controller and a Dynamic Trajectory Generator for the Franka Emika FR3 robot using the ros2_control framework and the Pinocchio kinematics library.

## Features

- **Real-Time Performance**: Optimized C++ logic designed for 1kHz control loops.
- **Adaptive Damping**: Employs a damped pseudo-inverse Jacobian with adaptive lambda based on manipulability to handle singular configurations.
- **Nullspace Projection**: Utilizes the robot's redundancy for joint limit avoidance without compromising the primary Cartesian task.
- **Safety Features**: Includes velocity command scaling, singularity thresholds, and low-pass filtering to ensure smooth hardware operation.
- **Quintic Trajectory Generation**: Standalone node for computing smooth S-curve motion profiles with dynamic duration based on Cartesian distance.

## Package Structure

- **src/cartesian_velocity_controller.cpp**: Implementation of the controller plugin.
- **src/trajectory_generator_node.cpp**: ROS 2 node for generating target poses.
- **include/**: Header files defining the controller and generator classes.
- **my_franka_controllers.xml**: Pluginlib export definitions.

## Control Architecture

The control law implemented is based on the following relationship:

dq_dot = J_inv_A * (K * delta_x) + (I - J_inv * J) * dq_null

- **Primary Task**: Minimizes Cartesian pose error using a proportional gain matrix (K).
- **Secondary Task**: Projects a joint-limit avoidance gradient into the nullspace of the Jacobian.
- **Scaling**: If any joint velocity exceeds hardware limits, the entire vector is scaled uniformly to maintain the direction of the end-effector motion.

## Installation

Building in Release mode is required to maintain the 1kHz cycle time:

```bash
colcon build --packages-select my_franka_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

1. Load the Controller
Ensure your robot description includes the controller configuration, then activate it:
```bash
ros2 control load_controller --set-state active my_cartesian_controller
```

2. Run the Trajectory Generator
```bash
ros2 run my_franka_controllers trajectory_generator_node
```

3. Send a Goal
Publish a PoseStamped message to the goal topic:
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 0.4, y: 0.0, z: 0.5}}}"
```

## Monitoring
The controller publishes tracking errors to:
```bash
/my_cartesian_controller/tracking_error
```

This can be monitored using PlotJuggler or standard ROS 2 echo commands to verify controller convergence.