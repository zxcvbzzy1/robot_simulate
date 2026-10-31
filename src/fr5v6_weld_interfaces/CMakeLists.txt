cmake_minimum_required(VERSION 3.8)
project(fr5v6_weld_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SeamEdge.msg"
  "msg/SeamCandidate.msg"
  "msg/SampledPoint.msg"
  "msg/WeldPathPoint.msg"
  "msg/WeldPath.msg"
  "srv/DetectSeams.srv"
  "srv/GeneratePaths.srv"
  DEPENDENCIES geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
