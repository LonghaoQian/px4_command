std_msgs/Header header

## Command_ID for safety, the command ID should increase with every command sent.
uint32 Command_ID

## Payload Pose Setpoint
float32    x_ref         ## [m]
float32    y_ref         ## [m]
float32    z_ref         ## [m]
float32    roll_ref      ## [deg]
float32    pitch_ref     ## [deg]
float32    yaw_ref       ## [deg]


