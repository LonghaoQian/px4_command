std_msgs/Header header

float32  Integral_x    ## [N]
float32  Integral_y    ## [N]
float32  Integral_z    ## [N] 

## quadrotor attitude

float32  q_0     ## 
float32  q_1     ##
float32  q_2     ##
float32  q_3     ##

## cable swing angle

float32  r_x    ## [m]
float32  r_y    ## [m]

float32  v_x    ## [m]
float32  v_y    ## [m]

float32  pos_error_x ## [m]
float32  pos_error_y ## [m]
float32  pos_error_z ## [m]

float32  vel_error_x  ## [m/s]
float32  vel_error_y  ## [m/s]
float32  vel_error_z  ## [m/s]
## raw vision feedback
float32 Lb_x    ## [m]
float32 Lb_y    ## [m]
float32 Lb_z    ## [m]

float32 Lb_dot_x ## [m/s]
float32 Lb_dot_y ## [m/s]
float32 Lb_dot_z ## [m/s]

## payload position measurement from vision
float32  Lv_x    ## [m]
float32  Lv_y    ## [m]
float32  Lv_z    ## [m]
## payload position measurement from mocap
float32  Lm_x    ## [m]
float32  Lm_y    ## [m]
float32  Lm_z    ## [m]
## payload position measurement from vision
float32  Vpv_x   ## [m/s]
float32  Vpv_y   ## [m/s]
float32  Vpv_z   ## [m/s]
## payload position measurement from mocap
float32  Vpm_x   ## [m/s]
float32  Vpm_y   ## [m/s]
float32  Vpm_z   ## [m/s]
## command lift 
float32  fL_x     ##
float32  fL_y     ##
float32  fL_z     ##

float32  acc_x  ##
float32  acc_y  ##
float32  acc_z  ##

# vision stream flag
bool isvisualfeedbacknormal ## determine whether the vision node has published data
bool isvisualqualitygood    ## determine whethet the vision feedback has high quality
int32  signallosscounter