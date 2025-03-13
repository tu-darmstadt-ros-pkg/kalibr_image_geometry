cmake_minimum_required(VERSION 3.8)
project(extended_image_geometry_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExtendedCameraInfo.msg"
  DEPENDENCIES ${DEPENDENCIES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
