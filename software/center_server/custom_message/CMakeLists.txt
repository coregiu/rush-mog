cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SyncedSensorData.msg"
  "msg/TaskRequest.msg"
  "msg/ActionIntent.msg"
  "msg/ReverseOperation.msg"
  "msg/InferenceResult.msg"
  "srv/SetMode.srv"
  "srv/EmergencyStop.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES sensor_msgs nav_msgs trajectory_msgs audio_common_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()