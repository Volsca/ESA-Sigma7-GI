cmake_minimum_required(VERSION 3.8)
project(s7spaceship_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Only if you used Option B:
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  "msg/Pos6.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  # Only if you used Option B:
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
