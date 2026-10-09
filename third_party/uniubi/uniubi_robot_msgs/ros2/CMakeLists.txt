cmake_minimum_required(VERSION 3.8)
project(uniubi)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Header.msg"
  "msg/MotorHeader.msg"
  "msg/Quaternionf.msg"
  "msg/Vector3f.msg"
  "msg/IMUState.msg"
  "msg/MotorObserved.msg"
  "msg/PowerObserved.msg"
  "msg/MotionObserved.msg"
  "msg/MotionOdometry.msg"
  "msg/GEOGPoint.msg"
  "msg/GPSFrame.msg"
  "msg/UWBRawObserved.msg"
  "msg/SensorObserved.msg"
  "msg/RemoteControl.msg"
  "msg/EventMessage.msg"
  "srv/System.srv"
)

ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY ../idl/ DESTINATION share/${PROJECT_NAME}/native_idl)

ament_package()
