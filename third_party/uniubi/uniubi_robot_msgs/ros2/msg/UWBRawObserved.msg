# Translated from uniubi_robot_msgs/idl/SensorObserved.idl: uniubi::dds_::UWBRawObserved.
# UWBPairState: pair_state 取值
uint8 UWB_PAIR_NONE=0
uint8 UWB_IN_PAIRING=1
uint8 UWB_PAIR_SUCCESS=2
uint8 UWB_PAIR_FAILED=3

uint8 valid
uint8 pair_state
int16 rssi    # 信号强度 单位 dbm
uint16 pitch
uint16 azimuth
uint32 distance
uint32 beacon_id
