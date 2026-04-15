# Franka ROS 2 Controllers

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

$$\dot{q} = J^{\#}_{A} (K \Delta x) + (I - J^{\#} J) \dot{q}_{null}$$

- **Primary Task**: Minimizes Cartesian pose error using a proportional gain matrix (K).
- **Secondary Task**: Projects a joint-limit avoidance gradient into the nullspace of the Jacobian.
- **Scaling**: If any joint velocity exceeds hardware limits, the entire vector is scaled uniformly to maintain the direction of the end-effector motion.

## Installation

Building in Release mode is required to maintain the 1kHz cycle time:

```bash
colcon build --packages-select my_franka_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash