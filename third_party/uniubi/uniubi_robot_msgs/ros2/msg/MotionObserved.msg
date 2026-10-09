# Translated from uniubi_robot_msgs/idl/MotionObserved.idl: uniubi::msg::dds_::MotionObserved_.
uint32 MAX_MOTOR_NUM=16

IMUState imu
int32 motor_num
uint64 timestamp
MotorObserved[16] motor
PowerObserved power
