std_msgs/Header header

## quadrotor attitude

float32  q_0     ##
float32  q_1     ##
float32  q_2     ##
float32  q_3     ##

float32  r_jx    ## [m]
float32  r_jy    ## [m]

float32  v_jx    ## [m]
float32  v_jy    ## [m]

float32  L_measured ## [m] measured L from mocap system

## payload pos error

float32  pos_error_x ## [m]
float32  pos_error_y ## [m]
float32  pos_error_z ## [m]

float32  angle_error_x  ## 
float32  angle_error_y  ## 
float32  angle_error_z  ## 

## quadrotor euler angle

float32  Euler_roll     ##
float32  Euler_pitch    ##
float32  Euler_yaw      ##

float32  fLj_x  ## [N]
float32  fLj_y  ## [N]
float32  fLj_z  ## [N]

float32  Delta_jp_x ## [N]
float32  Delta_jp_y ## [N]
float32  Delta_jp_z ## [N]

float32  acc_x  ##
float32  acc_y  ##
float32  acc_z  ##

float32  rd_jx  ## 
float32  rd_jy  ##