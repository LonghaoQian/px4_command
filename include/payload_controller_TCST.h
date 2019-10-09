#ifndef PAYLOAD_CONTROLLER_TCST_H
#define PAYLOAD_CONTROLLER_TCST_H

#include <Eigen/Eigen>
#include <math.h>
#include <iostream>
#include <string>
#include <command_to_mavros.h>
#include <px4_command_utils.h>
#include <math_utils.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/ControlOutput.h>

using namespace std;

class payload_controller_TCST
{
    public:

        //Constructor
        payload_controller_TCST(char drone_ID[20]):
            payload_controller_nh("~")
        {
            // use drone ID to get the correct name for parameters
            //uav_pref  = uav_pref + "uav";
            //for (int i = 0; i<20;i++) {
            //    uav_pref = uav_pref + drone_ID[i];
            //}
            uav_pref = "uav";
            uav_pref = uav_pref + drone_ID[0];
            payload_controller_nh.param<float>(uav_pref + "_Pos_GNC/mass", Quad_MASS, 1.0);
            payload_controller_nh.param<float>(uav_pref + "_Pos_GNC/cablelength", Cable_Length, 1.0);

            payload_controller_nh.param<float>(uav_pref + "_Pos_GNC/TetherOffset_x", TetherOffset(0), 0.5);
            payload_controller_nh.param<float>(uav_pref + "_Pos_GNC/TetherOffset_y", TetherOffset(1), 0);
            payload_controller_nh.param<float>(uav_pref + "_Pos_GNC/TetherOffset_z", TetherOffset(2), 0);

            payload_controller_nh.param<float>(uav_pref + "_Pos_GNC/PayloadSharingPortion", PayloadSharingPortion, 0.5);
            payload_controller_nh.param<float>("Payload/mass", Payload_Mass, 1.0);

            kv<< 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;
            kR<< 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;
            Kphi <<1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;    
            payload_controller_nh.param<float>("Pos_GNC/kv_xy", kv(0,0), 0.2);
            payload_controller_nh.param<float>("Pos_GNC/kv_xy", kv(1,1), 0.2);
            payload_controller_nh.param<float>("Pos_GNC/Kv_z",  kv(2,2), 0.4);

            payload_controller_nh.param<float>("Pos_GNC/kR_xy" , kR(0,0), 0.2);
            payload_controller_nh.param<float>("Pos_GNC/kR_xy" , kR(1,1), 0.2);
            payload_controller_nh.param<float>("Pos_GNC/kR_z"  , kR(2,2), 0.4);

            payload_controller_nh.param<float>("Pos_GNC/kL", kL, 0.5);

            payload_controller_nh.param<float>("Pos_GNC/Kphi_xy",  Kphi(0,0), 1);
            payload_controller_nh.param<float>("Pos_GNC/Kphi_xy",  Kphi(1,1), 1);
            payload_controller_nh.param<float>("Pos_GNC/Kphi_z",   Kphi(2,2), 1);

            payload_controller_nh.param<float>("Limitne/pxy_error_max", pos_error_max[0], 0.6);
            payload_controller_nh.param<float>("Limitne/pxy_error_max", pos_error_max[1], 0.6);
            payload_controller_nh.param<float>("Limit/pz_error_max" ,   pos_error_max[2], 1.0);

            payload_controller_nh.param<float>("Limit/pxy_int_max"  ,  int_max[0], 0.5);
            payload_controller_nh.param<float>("Limit/pxy_int_max"  ,  int_max[1], 0.5);
            payload_controller_nh.param<float>("Limit/pz_int_max"   ,  int_max[2], 0.5);
            payload_controller_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            payload_controller_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            payload_controller_nh.param<float>("Pos_GNC/fp_max_x", fp_max(0),1);
            payload_controller_nh.param<float>("Pos_GNC/fp_max_y", fp_max(1),1);
            payload_controller_nh.param<float>("Pos_GNC/fp_max_z", fp_max(2),1);

            u_l = Eigen::Vector3f(0.0,0.0,0.0);
            u_d = Eigen::Vector3f(0.0,0.0,0.0);
            r_j = Eigen::Vector2f(0.0,0.0);
            v_j = Eigen::Vector2f(0.0,0.0);

            TetherOffsetCross = Hatmap(TetherOffset);

            R_IP<< 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;
            R_IPd<< 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;
            B_j<<1.0,0.0,
                 0.0,1.0,
                 0.0,0.0;
            Cable_Length_sq = Cable_Length * Cable_Length;
            TotalLiftedMass = Payload_Mass* PayloadSharingPortion + Quad_MASS;
            /*nodes receiving */

        }

        //Printf the controller parameter
        void printf_param();
        void printf_result();
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        px4_command::ControlOutput payload_controller(const px4_command::DroneState& _DroneState, 
                                                      const px4_command::TrajectoryPoint& _Reference_State, 
                                                      float dt);
        // control command
        Eigen::Vector3d accel_sp;
        px4_command::ControlOutput _ControlOutput;
    private:
        ros::NodeHandle payload_controller_nh; // ros node for loading parameters
        /*configuration parameters*/
        float Quad_MASS;
        float Payload_Mass;
        float TotalLiftedMass;
        float Cable_Length;
        float Cable_Length_sq;
        string uav_pref;
        Eigen::Vector3f TetherOffset;
        Eigen::Matrix3f TetherOffsetCross;
        float  PayloadSharingPortion;
        //Controller parameter for the control law
        Eigen::Matrix3f kv;
        Eigen::Matrix3f kR;
        Eigen::Matrix3f Kphi;
        float kL;
        // payload attitude and quadrotor relative state
        Eigen::Matrix3f R_IP;
        Eigen::Vector3f L_j_dot;//rotation speed of the cable tip
        Eigen::Vector3f L_j; // the cable vector
        Eigen::Vector2f r_j, v_j;
        Eigen::Matrix<float, 3,2> B_j;

        /*
        temp variables
        */
        Eigen::Vector3d AttitudeTargetEuler;
        Eigen::Quaterniond AttitudeTargetQuaterniond;
        Eigen::Vector4f AttitudeTargetQuaternionv;
        Eigen::Vector4f AttitudeQuaternionv;
        Eigen::Matrix3f R_IPd;
        Eigen::Vector3f Xp, Vp, Xj, Vj, Omega_p, Xpd;
        /* Error signals */
        Eigen::Vector3f pos_error;
        Eigen::Vector3f angle_error;
        // error constraint parameters:
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f angular_error_max;
        Eigen::Vector3f int_max;
        Eigen::Vector3f fp_max;
        float tilt_max;
        float int_start_error;
        // normalized control input
        Eigen::Vector3f u_l;
        Eigen::Vector3f u_d;

};

px4_command::ControlOutput payload_controller_TCST::payload_controller(
    const px4_command::DroneState& _DroneState, 
    const px4_command::TrajectoryPoint& _Reference_State, 
    float dt)
{
    /* step 1: convert roll pitch yaw command to rotation matrix command*/
    AttitudeTargetEuler(0) = (double)_Reference_State.roll_ref;
    AttitudeTargetEuler(1) = (double)_Reference_State.pitch_ref;
    AttitudeTargetEuler(2) = (double)_Reference_State.yaw_ref;
    AttitudeTargetQuaterniond = quaternion_from_rpy(AttitudeTargetEuler);
    AttitudeTargetQuaternionv(0) = (float)AttitudeTargetQuaterniond.w();
    AttitudeTargetQuaternionv(1) = (float)AttitudeTargetQuaterniond.x();
    AttitudeTargetQuaternionv(2) = (float)AttitudeTargetQuaterniond.y();
    AttitudeTargetQuaternionv(3) = (float)AttitudeTargetQuaterniond.z();
    R_IPd =  QuaterionToRotationMatrix(AttitudeTargetQuaternionv);

    for(int i = 0; i < 4 ;i++){
        AttitudeQuaternionv(i) = _DroneState.payload_quaternion[i];
    }

    R_IP = QuaterionToRotationMatrix(AttitudeQuaternionv);


    /*Step 2 calculate L_j and L_j_dot based on payload information feedback
    
    Xj = Xp + R_IPt_j + L_j -> L_j = Xj-Xp-R_IBt_j
    Vj = Vp -R_IPt_jx omega_p + L_j_dot -> L_j_dot = Vj-Vp + R_IPt_jx omega_p
    */
    // put states into vector array for easy calculations
    for (int i = 0; i < 3; i ++) {
        Xpd(i) = _Reference_State.position_ref[i];
        Xj(i) = _DroneState.position[i];
        Xp(i) = _DroneState.payload_pos[i];
        Vp(i) = _DroneState.payload_vel[i];
        Vj(i) = _DroneState.velocity[i];
        Omega_p(i) = _DroneState.payload_angular_vel[i];
    }
    L_j = Xj - Xp - R_IP * TetherOffset;
    L_j_dot = Vj- Vp + R_IP * TetherOffsetCross* Omega_p;

    r_j(0) = L_j(0);
    r_j(1) = L_j(1);
    v_j(0) = L_j_dot(0);
    v_j(1) = L_j_dot(1);

    float sq_r = r_j(0)*r_j(0) + r_j(1)*r_j(1);

    if (Cable_Length_sq - sq_r>0.01)
    {
        B_j(2,0) =  -r_j(0)/sqrt((Cable_Length_sq - sq_r));
        B_j(2,1) =  -r_j(1)/sqrt((Cable_Length_sq - sq_r));
    }else{
        B_j(2,0) = -0.1;
        B_j(2,1) = -0.1;
    }

    /*Step 3 calculate payload position and attitude error*/
    pos_error = Xp-Xpd;
    float scale_p = sqrt(1+ pos_error.transpose() * pos_error);
    pos_error = pos_error/scale_p;

    angle_error = 0.5* Veemap(R_IPd.transpose()*R_IP- R_IP.transpose() * R_IPd);

    /* put a hard constraint on the angle_error */
    
    //or (int i = 0; i<3;i++) {
    //    if (angle_error(i)>angular_error_max(i)) {
    //        angle_error(i) = angular_error_max(i);
     //   }    
    //}

    /*Step 4 calculate control force  control law form the GNC 2019 paper*/
    Eigen::Vector3f U = Vp+kv*pos_error - R_IP * TetherOffsetCross*(Omega_p + kR*angle_error) + B_j*(v_j + kL*r_j);
    for (int i = 0; i < 3; i++)
    {
        U(i) =  constrain_function(U(i), fp_max(i));
        u_d(i) = 0;
        // additional feedback based on payload relative position:
    }
    u_l = - Kphi * (Vj+kv*pos_error -  kR * R_IP * TetherOffsetCross * angle_error + kL*B_j*r_j);

    /*Step 5 calculate Delta_j_hat based on quadrotor acceleration feedback */

    /*Step 6 calculate Delta_T and Delta_R based on last control force */

    /*TO DO use intergral to facilitate robust control*/
    /*for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) < int_start_error)
        {
            //integral[i] += pos_error[i] * dt;
            integral[i] += ( -Kp[i]*pos_error[i] - Kv[i] * vel_error[i]) * dt;

            if(abs(integral[i]) > int_max[i])
            {
                cout << "Integral saturation! " << " [0-1-2] "<< i <<endl;
                cout << "[integral]: "<< integral[i]<<" [int_max]: "<<int_max[i]<<" [m/s] "<<endl;
            }

            integral[i] = constrain_function(integral[i], int_max[i]);
        }else
        {
            integral[i] = 0;
        }

        // If not in OFFBOARD mode, set all intergral to zero.
        if(_DroneState.mode != "OFFBOARD")
        {
            integral[i] = 0;
        }
        u_d[i] = constrain_function(u_d[i], int_max[i]);
    }*/


    // desired acceleration
    accel_sp[0] = u_l[0] - u_d[0];
    accel_sp[1] = u_l[1] - u_d[1];
    accel_sp[2] = u_l[2] - u_d[2] + 9.8;
    
    // desired thrust  = desired accel
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    thrust_sp =  px4_command_utils::accelToThrust(accel_sp, TotalLiftedMass, tilt_max);
    throttle_sp = px4_command_utils::thrustToThrottle(thrust_sp);

    for (int i=0; i<3; i++)
    {
        _ControlOutput.u_l[i] = u_l[i];
        _ControlOutput.u_d[i] = u_d[i];
        _ControlOutput.Thrust[i]   = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }

    return _ControlOutput;

}

void payload_controller_TCST::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  TCST 2019 Paylaod Pose Controller  <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;
    //cout << "int [X Y Z] : " << integral[0] << " [N] "<< integral[1]<<" [N] "<<integral[2]<<" [N] "<<endl;
    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;
    cout << "r_j [X Y] : " << r_j(0) << " [m] " << r_j(1) << " [m] " <<endl;
    cout << "v_j [X Y] : " << v_j(0) << " [m/s] "<< v_j(1) << " [m/s] " << endl;
    cout << "pos_error [X Y Z] : " << pos_error[0] << " [m] " <<  pos_error[1] << " [m] " <<  pos_error[2] << " [m] " << endl;
    cout << "angle_error [X Y Z] : " << angle_error[0] << " [] "<< angle_error[1] << " [] " << angle_error[2] << " [] " <<endl;

    // verify the target quaternion has been calculated:
    Eigen::Vector3d Euler_Target = quaternion_to_euler2(AttitudeTargetQuaternionv);
    cout << "Euler_Target_roll : "  << Euler_Target(0)*57.3 << " [DEG] ";
    cout << "Euler_Target_pitch : " << Euler_Target(1)*57.3 << " [DEG] ";;
    cout << "Euler_Target_yaw : "   << Euler_Target(2)*57.3 << " [DEG] ";;
    cout << endl;

    // verify that the 
    Eigen::Vector3d Euler = quaternion_to_euler2(AttitudeQuaternionv);
    cout << "Euler_roll : "  << Euler(0)*57.3 << " [DEG] ";
    cout << "Euler_pitch : " << Euler(1)*57.3 << " [DEG] ";
    cout << "Euler_yaw : "   << Euler(2)*57.3 << " [DEG] ";
    cout << endl;
}

// print out controller parameters
void payload_controller_TCST::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Payload control method in TCST 2019 paper (Parameter)  <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout <<"UAV ID : "<<uav_pref<<endl;
    cout <<"Quad_MASS : "<< Quad_MASS << endl;
    cout <<"Payload_MASS : "<< Payload_Mass << endl;
    cout <<"Cable_Length : "<< Cable_Length << endl;
    cout <<"Tether Offset x : "<< TetherOffset(0) << " [m] ";
    cout <<"Tether Offset y : "<< TetherOffset(1) << " [m] ";
    cout <<"Tether Offset z : "<< TetherOffset(2) << " [m] ";
    cout << endl;
    cout <<"a_j : " <<PayloadSharingPortion<<endl;
    cout <<"kv_x : "<< kv(0,0) << endl;
    cout <<"kv_y : "<< kv(1,1) << endl;
    cout <<"kv_z : "<< kv(2,2) << endl;

    cout <<"kR_x : "<< kR(0,0)<<endl;
    cout <<"kR_y : "<< kR(1,1)<<endl;
    cout <<"kR_z : "<< kR(2,2)<<endl;

    cout <<"kphi_x : " << Kphi(0,0) <<endl;
    cout <<"kphi_y : " << Kphi(1,1) <<endl;
    cout <<"kphi_z : " << Kphi(2,2) <<endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;

    //cout <<"pxy_int_max : "<< int_max[0] << endl;
    //cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    //cout <<"int_start_error : "<< int_start_error << endl;

    cout << "fpmax_x : " << fp_max(0) << "fpmax_y : " << fp_max(1) << "fpmax_z : " << fp_max(2) <<endl;
}

#endif
