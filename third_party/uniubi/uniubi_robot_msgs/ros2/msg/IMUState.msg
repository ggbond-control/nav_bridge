# Translated from uniubi_robot_msgs/idl/MotionObserved.idl: uniubi::dds_::IMUState.
float32 temp           # 温度 ℃
Vector3f accel         # 加速度 m/s² (x/y/z)
Vector3f gyro          # 角速度 rad/s (x/y/z)
Vector3f mag           # 磁场 μT (x/y/z)
Vector3f euler         # 欧拉角 rad (x=roll[-π,π] / y=pitch[-π/2,π/2] / z=yaw[-π,π])
Quaternionf quaternion # 姿态四元数,无量纲
