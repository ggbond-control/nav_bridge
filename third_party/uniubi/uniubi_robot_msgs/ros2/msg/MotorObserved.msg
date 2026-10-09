# Translated from uniubi_robot_msgs/idl/MotionObserved.idl: uniubi::dds_::MotorObserved.
# MotorDeviceErrno: error 取值
uint8 MOTOR_NORMAL=0
uint8 MOTOR_PRE_DRIVER=1
uint8 MOTOR_ECODE_ERROR=2
uint8 MOTOR_OVER_SPEED=3
uint8 MOTOR_OVER_TEMPE=4
uint8 MOTOR_OVER_CURRENT=5
uint8 MOTOR_OVER_VOLTAGE=6
uint8 MOTOR_PG_ABNORMALITY=59
uint8 MOTOR_HW_UNDERVOLTAGE=60
uint8 MOTOR_COMM_ERROR=63
uint8 MOTOR_CONTROL_OFFLINE=64
uint8 CONTROL_MOTOR_NOT_ENABLE=65
uint8 MOTOR_CONTROL_NOT_READY=66
uint8 MOTOR_CONTROL_UPGRADE=67
uint8 MOTOR_NO_CALIBRATED=68
uint8 MOTOR_URDF_NOT_MAPPED=69

uint8 enable
uint8 online
uint8 error    # 见 MotorDeviceErrno
float32 position
float32 velocity
float32 torque
float32 temp
float32 voltage
float32 loss_rate
float32 max_torque
MotorHeader header
