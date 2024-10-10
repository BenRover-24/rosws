cmake_minimum_required(VERSION 3.5)
project(test_sim)

# Trouver les paquets nécessaires
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_library(test_sim_plugin SHARED
  src/servo_sim_plugin.cpp
)
target_link_libraries(test_sim_plugin
  gazebo_ros::gazebo_ros
)

ament_target_dependencies(test_sim_plugin rclcpp gazebo_msgs std_msgs) 

install(TARGETS test_sim_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Exporter le plugin Gazebo
gazebo_ros_add_plugin_library(test_sim_plugin)

# Installer le fichier URDF de simulation
install(FILES
  launch/test_sim.launch.py
  urdf/test_sim.urdf
  DESTINATION share/${PROJECT_NAME}
)