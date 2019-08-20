##
gnome-terminal --window -e 'bash -c "roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
