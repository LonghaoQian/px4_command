std_msgs/Header header

## simplified log
## drone status
bool     connected             ## 机载电脑是否连接上飞控，true已连接，false则不是
bool     armed                 ## 是否解锁，true为已解锁，false则不是
string   mode                  ## PX4飞控当前飞行模式
float32  time_from_start       ## [s] ## 系统启动时间
bool     mocapOK               ## Flag showing whether the mocap msg has been updated very loop

ControlCommand Control_Command ## control command 

float32[3] Throttle            ## throttle calculated by controller.
