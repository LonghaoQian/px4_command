#!/bin/bash

# Open the first tab (default terminal window)
gnome-terminal -- bash -c "roslaunch px4 multi_drone_test.launch; exec bash"

# Wait for the terminal to open
sleep 1

# Use xdotool to open additional tabs in the same terminal
xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 5; roslaunch optitrack_broadcast emulator_for_gazebo.launch; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 2; rosrun qt_ground_station qt_ground_station; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 3; roslaunch px4_command px4_interdrone_communication_sitl.launch paramfile:=Parameter_for_control_2_drone.yaml; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 3; roslaunch px4_command px4_multidrone_pos_estimator_pure_vision.launch uavID:=uav0; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 3; roslaunch px4_command px4_multidrone_pos_controller_sitl.launch uavID:=uav0 paramfile:=Parameter_for_control_2_drone.yaml; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 3; roslaunch px4_command px4_multidrone_pos_estimator_pure_vision.launch uavID:=uav1; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 3; roslaunch px4_command px4_multidrone_pos_controller_sitl.launch uavID:=uav1 paramfile:=Parameter_for_control_2_drone.yaml; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 2; rosrun px4_command set_uav0_mode; exec bash"
xdotool key Return

xdotool key ctrl+shift+t
xdotool type --delay 1 -- "sleep 2; rosrun px4_command set_uav1_mode; exec bash"
xdotool key Return
