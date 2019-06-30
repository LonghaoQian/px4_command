std_msgs/Header header

## 位置控制器输出： 期望加速度、期望升力、期望角度、期望角速度、期望角加速度
float32[3] desired_acceleration        ## [m/s^2]
float32[3] desired_thrust              ## [N]
float32[3] desired_thrust_normalized   ## [0-1] 惯性系
float32 desired_throttle               ## [0-1] 机体系z轴
float32[3] desired_attitude            ## [rad]
geometry_msgs/Quaternion desired_att_q ## 四元数
