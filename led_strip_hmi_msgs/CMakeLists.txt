cmake_minimum_required(VERSION 3.5)
project(led_strip_hmi_msgs VERSION 0.1.0)

# find ament and rosidl message generation machinery
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find dependencies for message fields
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# generate the message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LEDStripPhysicalSegment.msg"
  "msg/LEDStripPhysicalConfig.msg"
  "msg/LEDStripPhysicalConfigArray.msg"
  "msg/LEDStripEffect.msg"
  "msg/LEDStripProjectionInfo.msg"
  "msg/LEDStripProjectionInfoArray.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# export the rosidl runtime dependency for downstream packages
ament_export_dependencies(rosidl_default_runtime)

# ament package macro
ament_package()