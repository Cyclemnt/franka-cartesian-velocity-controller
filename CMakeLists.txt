cmake_minimum_required(VERSION 3.8)
project(my_franka_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/cartesian_velocity_controller.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)

target_include_directories(trajectory_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS 2 dependencies (Standard ROS 2 Macro)
# This handles the headers and linking for all ROS-based packages correctly.
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  geometry_msgs
  sensor_msgs
  realtime_tools
  pluginlib
)

ament_target_dependencies(trajectory_generator_node
  rclcpp
  geometry_msgs
  tf2_ros 
  tf2_eigen 
  Eigen3
)

# Link External Math Libraries
# We do NOT use "PUBLIC" or "PRIVATE" here to match the 
# "plain" signature used by ament_target_dependencies above.
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_link_libraries(trajectory_generator_node
  ${PROJECT_NAME}
)

# Plugin Registration and Installation
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS trajectory_generator_node DESTINATION lib/${PROJECT_NAME})
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()