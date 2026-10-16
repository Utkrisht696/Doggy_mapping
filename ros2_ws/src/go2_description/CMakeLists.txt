cmake_minimum_required(VERSION 3.8)
project(go2_description)
find_package(ament_cmake REQUIRED)
# ROS 2 packaging wrapper only. Keep the developer URDF and assets unmodified.
set(UNITREE_DESCRIPTION "${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk/unitree_ros/robots/go2_description")
install(DIRECTORY "${UNITREE_DESCRIPTION}/urdf" "${UNITREE_DESCRIPTION}/dae"
  "${UNITREE_DESCRIPTION}/meshes" "${UNITREE_DESCRIPTION}/xacro"
  DESTINATION share/${PROJECT_NAME})
ament_package()
