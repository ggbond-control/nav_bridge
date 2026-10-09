# Translated from uniubi_robot_msgs/idl/MotionObserved.idl: uniubi::dds_::Vector3f.
# IMUDeviceErrno: error 取值
int8 IMU_NORMAL=0
int8 IMU_INVALID=1
int8 IMU_CONTROL_OFFLINE=64
int8 IMU_CONTROL_NOT_READY=65
int8 IMU_CONTROL_UPGRADE=66
int8 IMU_CONTROL_NOT_PARAMS=67
int8 IMU_NOT_READY=68

int8 error    # 见 IMUDeviceErrno
float32 x
float32 y
float32 z
