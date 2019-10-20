#!/bin/bash

xterm -e "roslaunch px4_command mavros_multi_drone.launch uavID:=uav0" &
xterm -e "roslaunch px4_command px4_multidrone_pos_estimator_pure_vision.launch uavID:=uav0" &
xterm -e "roslaunch px4_command px4_multidrone_pos_controller.launch uavID:=uav0"
