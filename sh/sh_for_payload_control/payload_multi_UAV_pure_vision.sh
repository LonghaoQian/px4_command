##
gnome-terminal --window -e 'bash -c "roslaunch px4_command mavros_multi_drone.launch uavID:=uav0; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_command px4_multidrone_pos_estimator_pure_vision.launch uavID:=uav0; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_command px4_multidrone_pos_controller_gazebo.launch uavID:=uav0; exec bash"' \
