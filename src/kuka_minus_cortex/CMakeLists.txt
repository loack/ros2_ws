cmake_minimum_required(VERSION 3.5)
project(kuka_minus_cortex)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install Python modules
install(DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME})

# Export package
ament_export_dependencies(rclcpp urdf xacro rviz2)
ament_package()