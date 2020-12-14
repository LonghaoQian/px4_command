std_msgs/Header header

## the estimated disturbances
float32  W_x    ## [N]
float32  W_y    ## [N]
float32  W_z    ## [N] 

## quadrotor attitude
float32  q_0     ## 
float32  q_1     ##
float32  q_2     ##
float32  q_3     ##

## cable motion
float32  r_x    ## [m]
float32  r_y    ## [m]

float32  v_x    ## [m]
float32  v_y    ## [m]

## tracking errors
float32  pos_error_x ## [m]
float32  pos_error_y ## [m]
float32  pos_error_z ## [m]

float32  vel_error_x  ## [m/s]
float32  vel_error_y  ## [m/s]
float32  vel_error_z  ## [m/s]

## payload position measurement from mocap
float32  Lm_x    ## [m]
float32  Lm_y    ## [m]
float32  Lm_z    ## [m]

## payload position measurement from mocap
float32  Vpm_x   ## [m/s]
float32  Vpm_y   ## [m/s]
float32  Vpm_z   ## [m/s]

## the  command lift 
float32  fL_x     ##
float32  fL_y     ##
float32  fL_z     ##

## the quadrotor inerital acceleration
float32  acc_x  ##
float32  acc_y  ##
float32  acc_z  ##