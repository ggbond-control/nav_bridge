# Translated from uniubi_robot_msgs/idl/SensorObserved.idl: uniubi::dds_::GPSFrame.
# GPSSignalLevel: level 取值
int32 GPS_GRE_38DB=0
int32 GPS_GRE_30DB=1
int32 GPS_LES_30DB=2

uint32 valid
float32 speed
int32 level
int32 rssi    # 信号强度原始值 单位 dbm
GEOGPoint point
