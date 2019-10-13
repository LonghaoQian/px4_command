std_msgs/Header header

## 机载电脑是否连接上飞控，true已连接，false则不是
bool connected
## 是否解锁，true为已解锁，false则不是
bool armed
## PX4飞控当前飞行模式
string mode

## 系统启动时间
float32 time_from_start             ## [s]

## 无人机状态量：位置、速度、姿态
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## []
float32[3] attitude_rate            ## [rad/s]
float32[3] acceleration             ## [m/s^2] defined in 

float32[3] payload_vel              ## [m/s]
float32[3] payload_pos              ## [m]
float32[4] payload_quaternion       ## []
float32[3] payload_angular_vel      ## [rad/s]
## Flag showing whether the mocap msg has been updated very loop
bool mocapOK                        