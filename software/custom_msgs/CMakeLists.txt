cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "msg/SyncedSensorData.msg"
  "msg/RobotState.msg"
  "msg/TaskRequest.msg"
  "msg/ActionIntent.msg"
  "msg/ReverseOperation.msg"
  "msg/InferenceResult.msg"
  "msg/SafetyState.msg"
  "msg/SystemStatus.msg"
  "msg/MapInfo.msg"
  "msg/Frontier.msg"
  "msg/FrontierList.msg"
  "msg/TargetDetection.msg"
  "msg/TargetList.msg"
  "msg/LocalizationStatus.msg"
  "msg/NavStatus.msg"
  "msg/SearchStatus.msg"
  "msg/MissionStatus.msg"
  "msg/VelocityMuxStatus.msg"
  "msg/DeviceHealth.msg"
  "msg/HealthStatus.msg"
  "srv/SetMode.srv"
  "srv/EmergencyStop.srv"
  "srv/MapControl.srv"
  "srv/SetInitialPose.srv"
  "srv/HealthCheck.srv"
  "action/ExecuteTask.action"
  "action/NavigateTo.action"
  "action/Explore.action"
  "action/SearchObject.action"
  "action/ExecuteMission.action"
  DEPENDENCIES std_msgs sensor_msgs nav_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
