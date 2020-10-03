/***************************************************************************************************************************
* outdoorwifitest.cpp
*
* Author: Longhao Qian
*
* Update Time: 2020.10.03
*
* Introduction:  An outdoor wifi communincation testing tool. It sends out a fake drone state and a fake
* auxiliary state at 50Hz to the ground station to test the wifi quality
* computer
* This node should run on the onboard computer of the drone
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <px4_command/AuxiliaryState.h>
#include <px4_command/FleetStatus.h>
#include <px4_command_utils.h>
int main(int argc, char **argv) {
    ros::init(argc, argv, "outdoorwifitest");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    float CurrentTime = 0.0;
    ros::Time begin_time = ros::Time::now();

    px4_command::AuxiliaryState FakeAuxstate;
    px4_command::FleetStatus    FaKeFleetStatus;

    ros::Publisher pubFakeAuxiliaryState = nh.advertise<px4_command::AuxiliaryState> ("/test/px4_command/auxiliarystate", 1000);
    ros::Publisher pubFakeFleetStatus = nh.advertise<px4_command::FleetStatus> ("/test/px4_command/fleetstatus", 1000);

    int tickfordisplay = 0;
    int ticknow = 0;
    bool displaytimestamp = false;
    int timedispstep = 10;

    while(ros::ok()) {

        FakeAuxstate.header.stamp = ros::Time::now();
        FakeAuxstate.L_measured = 1.0;

        FakeAuxstate.q_0 = 0.3;
        FakeAuxstate.q_1 = 0.1;
        FakeAuxstate.q_2 = -0.2;
        FakeAuxstate.q_3 = 0.8;

        FakeAuxstate.r_jx = 0.2;
        FakeAuxstate.r_jy = -0.3;

        FakeAuxstate.v_jx = 0.2;
        FakeAuxstate.v_jy = 0.1;

        FakeAuxstate.pos_error_x = 0.1;
        FakeAuxstate.pos_error_y = 0.1;
        FakeAuxstate.pos_error_z = 0.1;

        FakeAuxstate.angle_error_x =  0.1;
        FakeAuxstate.angle_error_y =  0.1;
        FakeAuxstate.angle_error_z =  0.1;
       
        FakeAuxstate.Euler_roll =   3.0;
        FakeAuxstate.Euler_pitch =  4.0;
        FakeAuxstate.Euler_yaw  =  20.0;

        FakeAuxstate.fLj_x  = 20.0;
        FakeAuxstate.fLj_y  = -1.0;
        FakeAuxstate.fLj_z  = 2.0;

        FakeAuxstate.Delta_jp_x = 1.0;
        FakeAuxstate.Delta_jp_y = -4.0;
        FakeAuxstate.Delta_jp_z = 2.0;

        FakeAuxstate.acc_x = 0.4;
        FakeAuxstate.acc_y = 0.5;
        FakeAuxstate.acc_z = 0.3;

        FakeAuxstate.rd_jx = 0.4;
        FakeAuxstate.rd_jy = 0.1;

        pubFakeAuxiliaryState.publish(FakeAuxstate);

        FaKeFleetStatus.r_jx  = 0.2;
        FaKeFleetStatus.r_jy  = 0.1;
        FaKeFleetStatus.v_jx  = 0.2;
        FaKeFleetStatus.v_jy  = 0.1;
        FaKeFleetStatus.f_Ljx = 0.1;
        FaKeFleetStatus.f_Ljy = 0.1;
        FaKeFleetStatus.f_Ljz = 0.2;

        FaKeFleetStatus.delta_jx = 0.4;
        FaKeFleetStatus.delta_jy = 0.3;
        FaKeFleetStatus.delta_jz = 0.1;

        FaKeFleetStatus.rd_jx = 0.2;
        FaKeFleetStatus.rd_jy = 0.1;
        FaKeFleetStatus.emergency = true;

        pubFakeFleetStatus.publish(FaKeFleetStatus);

        CurrentTime = px4_command_utils::get_time_in_sec(begin_time);
        
        ticknow = int(floor(CurrentTime)); // record the tick
        if((ticknow % timedispstep == 0)) { // ticks now is a multiple of 10
            if(tickfordisplay == ticknow){ 
                // if the display tick is equal to the current tick, display time and set the advance the tickfordisplay by 1
                ROS_INFO("Wifi testing node is running... Time stamp: %f [s]",CurrentTime); 
                tickfordisplay += timedispstep;
            }
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
