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
#include <px4_command/AddonForce.h>
#include <px4_command/FleetStatus.h>
#include <vector>
using std::string;
using std::vector;
class payload_controller_TCST
{
    public:
        payload_controller_TCST(char drone_ID[20],
                                ros::NodeHandle& main_handle) {
            // use drone ID to get the correct name for parameters
            uav_pref = "uav";
            uav_pref = uav_pref + drone_ID[0];
            main_handle.param<float>(uav_pref + "_Pos_GNC/mass"       , Quad_MASS, 1.0);
            main_handle.param<float>(uav_pref + "_Pos_GNC/cablelength", Cable_Length, 1.0);
            main_handle.param<float>(uav_pref + "_Pos_GNC/TetherOffset_x", TetherOffset(0), 0.5);
            main_handle.param<float>(uav_pref + "_Pos_GNC/TetherOffset_y", TetherOffset(1), 0);
            main_handle.param<float>(uav_pref + "_Pos_GNC/TetherOffset_z", TetherOffset(2), 0);
            main_handle.param<float>(uav_pref + "_Pos_GNC/PayloadSharingPortion", PayloadSharingPortion, 0.5);
            main_handle.param<float>("Payload/mass"                             , Payload_Mass,          1.0);

            kv   << 1.0,0.0,0.0,
                    0.0,1.0,0.0,
                    0.0,0.0,1.0;
            kR   << 1.0,0.0,0.0,
                    0.0,1.0,0.0,
                    0.0,0.0,1.0;
            Kphi << 1.0,0.0,0.0,
                    0.0,1.0,0.0,
                    0.0,0.0,1.0;
                
            main_handle.param<float>("Pos_GNC/kv_xy",  kv(0,0), 0.2);
            main_handle.param<float>("Pos_GNC/kv_xy",  kv(1,1), 0.2);
            main_handle.param<float>("Pos_GNC/Kv_z" ,  kv(2,2), 0.4);
            main_handle.param<float>("Pos_GNC/kR_xy" , kR(0,0), 0.2);
            main_handle.param<float>("Pos_GNC/kR_xy" , kR(1,1), 0.2);
            main_handle.param<float>("Pos_GNC/kR_z"  , kR(2,2), 0.4);
            main_handle.param<float>("Pos_GNC/kL"    , kL, 0.5);

            main_handle.param<float>("Pos_GNC/Kphi_xy",   Kphi(0,0), 1);
            main_handle.param<float>("Pos_GNC/Kphi_xy",   Kphi(1,1), 1);
            main_handle.param<float>("Pos_GNC/Kphi_z" ,   Kphi(2,2), 1);

            main_handle.param<float>("Limitne/pxy_error_max", pos_error_max[0], 0.6);
            main_handle.param<float>("Limitne/pxy_error_max", pos_error_max[1], 0.6);
            main_handle.param<float>("Limit/pz_error_max" ,   pos_error_max[2], 1.0);

            main_handle.param<float>("Limit/pxy_int_max"  ,   int_max[0], 0.5);
            main_handle.param<float>("Limit/pxy_int_max"  ,   int_max[1], 0.5);
            main_handle.param<float>("Limit/pz_int_max"   ,   int_max[2], 0.5);
            main_handle.param<float>("Limit/tilt_max"     ,   tilt_max, 20.0);
            main_handle.param<float>("Limit/int_start_error", int_start_error, 0.3);

            main_handle.param<float>("Pos_GNC/fp_max_x", fp_max(0),1);
            main_handle.param<float>("Pos_GNC/fp_max_y", fp_max(1),1);
            main_handle.param<float>("Pos_GNC/fp_max_z", fp_max(2),1);

            main_handle.param<int>("Pos_GNC/num_drone", num_of_drones,1);

            u_l = Eigen::Vector3f(0.0,0.0,0.0);// norminal control
            u_d = Eigen::Vector3f(0.0,0.0,0.0);// disturbance estimation
            r_j = Eigen::Vector2f(0.0,0.0);    // relative position
            v_j = Eigen::Vector2f(0.0,0.0);    // realtive velocity

            TetherOffsetCross = Hatmap(TetherOffset);

            R_IP<< 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;
            R_IPd<< 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0;
            B_j<<  1.0,0.0,
                   0.0,1.0,
                   0.0,0.0;
            Cable_Length_sq = Cable_Length * Cable_Length;
            TotalLiftedMass = Payload_Mass* PayloadSharingPortion + Quad_MASS;
            /* initialize publisher and subscriber */
            pubFleetState = main_handle.advertise<px4_command::FleetStatus> ("/" + uav_pref + "/px4_command/fleetstatus", 
                                                                            1000);
            subAddonForce = main_handle.subscribe<px4_command::AddonForce>  ("/uav0/px4_command/addonforce", 
                                                                            1000,
                                                                            &payload_controller_TCST::GetAddonForce, 
                                                                            this);
            //clientSendParameter =  = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
            /* initialize the disturbance estimation */
            Delta_pt << 0.0,
                       0.0,
                       0.0;
            Delta_rt<< 0.0,
                      0.0,
                      0.0;
            Delta_j<< 0.0,
                      0.0,
                      0.0;
            Identity<<1.0,0.0,0.0,
                      0.0,1.0,0.0,
                      0.0,0.0,1.0;
            g_I << 0.0,
                   0.0,
                   -9.81;
            D << 0.0,0.0,0.0,
                 0.0,0.0,0.0,
                 0.0,0.0,0.0;
            // calculate E_j
            Eigen::Vector3f temp_t_j;
            float           temp_a_j;
            string          temp_uav_pref;
            for (int i = 0; i < num_of_drones ; i ++) {
                // reset names:
                temp_uav_pref = "uav";
                temp_uav_pref = temp_uav_pref + to_string(i);
                main_handle.param<float>(temp_uav_pref+"_Pos_GNC/TetherOffset_x", temp_t_j(0), 0.5);
                main_handle.param<float>(temp_uav_pref+"_Pos_GNC/TetherOffset_y", temp_t_j(1), 0);
                main_handle.param<float>(temp_uav_pref+"_Pos_GNC/TetherOffset_z", temp_t_j(2), 0); 
                main_handle.param<float>(temp_uav_pref+"_Pos_GNC/PayloadSharingPortion", temp_a_j, 0); 
                D += temp_a_j * Hatmap(temp_t_j)* Hatmap(temp_t_j);
            }
            E_j = PayloadSharingPortion * TetherOffsetCross * D.inverse();


            ParamSrv.request.controllername = uav_pref + " Payload Pos_GNC";
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
        // add-on force and disturbance estimation
        px4_command::ControlOutput _ControlOutput;
        void GetAddonForce(const px4_command::AddonForce::ConstPtr &msg);
        void Sendfleetstate();
    private:
        ros::Publisher             pubFleetState;
        ros::Subscriber            subAddonForce;
        ros::ServiceClient   clientSendParameter;
        px4_command::ControlParameter ParamSrv;
        px4_command::AddonForce    _AddonForce;
        px4_command::FleetStatus   _FleetStatus;
        /*drone state*/
        Eigen::Matrix3f R_Ij;
        Eigen::Vector4f quaternion_q;
        Eigen::Vector3f omega_q;
        /*configuration parameters*/
        int num_of_drones;
        float Quad_MASS;
        float Payload_Mass;
        float TotalLiftedMass;
        float Cable_Length;
        float Cable_Length_sq;
        string uav_pref;
        Eigen::Vector3f TetherOffset;
        Eigen::Matrix3f TetherOffsetCross;
        float  PayloadSharingPortion; // = aj in the TCST paper
        Eigen::Vector3f g_I;
        Eigen::Matrix3f E_j;
        Eigen::Matrix3f D;
        //Controller parameter for the control law
        float lambda_j;
        Eigen::Matrix3f kv;
        Eigen::Matrix3f kR;
        Eigen::Matrix3f Kphi;
        float kL;
        Eigen::Matrix3f Identity;
        // payload attitude and quadrotor relative state
        Eigen::Matrix3f R_IP;
        Eigen::Vector3f L_j_dot;//rotation speed of the cable tip
        Eigen::Vector3f L_j; // the cable vector
        Eigen::Vector2f r_j, v_j;
        Eigen::Matrix<float, 3,2> B_j;
        Eigen::Matrix<float, 3,3> BB_j;
        // distubance on quadrotor
        Eigen::Vector3f Delta_pt;
        Eigen::Vector3f Delta_rt;
        Eigen::Vector3f Delta_j;
        Eigen::Vector3f f_L_j;
        // cross feeding terms
        Eigen::Vector3f R1;
        Eigen::Vector3f R2;
        /*
        temp variables
        */
        Eigen::Vector3d    AttitudeTargetEuler;
        Eigen::Quaterniond AttitudeTargetQuaterniond;
        Eigen::Vector4f    AttitudeTargetQuaternionv;
        Eigen::Vector4f    AttitudeQuaternionv;
        Eigen::Matrix3f    R_IPd;
        Eigen::Vector3f    Xp, Vp, Xj, Vj, Omega_p, Xpd;
        Eigen::Matrix2f    BB_temp;
        Eigen::Matrix2f    BB_inverse;
        Eigen::Vector3f    Delta_j_p;
        Eigen::Vector3f    dot_vqj;
        Eigen::Vector3f    accel_body;
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
        // 归一化推力 ： 根据电机模型，反解出归一化推力
        Eigen::Vector3d thrust_sp;
        Eigen::Vector3d throttle_sp;
};

px4_command::ControlOutput payload_controller_TCST::payload_controller(
    const px4_command::DroneState&      _DroneState, 
    const px4_command::TrajectoryPoint& _Reference_State, 
    float dt)
{
    /* step 1: convert roll pitch yaw command to rotation matrix command*/
    AttitudeTargetEuler(0)       = (double)_Reference_State.roll_ref;
    AttitudeTargetEuler(1)       = (double)_Reference_State.pitch_ref;
    AttitudeTargetEuler(2)       = (double)_Reference_State.yaw_ref;
    AttitudeTargetQuaterniond    = quaternion_from_rpy(AttitudeTargetEuler);
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
    
    // get quadrotor rotation matrix
    quaternion_q(0) = _DroneState.attitude_q.w;
    quaternion_q(1) = _DroneState.attitude_q.x;
    quaternion_q(2) = _DroneState.attitude_q.y;
    quaternion_q(3) = _DroneState.attitude_q.z;
    R_Ij = QuaterionToRotationMatrix(quaternion_q);
    // get quadrotor acceleration in inertial frame
    for( int i = 0; i < 3 ; i ++) {
        omega_q(i)    = _DroneState.attitude_rate[i];
        accel_body(i) = _DroneState.acceleration[i];
    }
    //dot_vqj = R_Ij * Hatmap(omega_q) * R_Ij.transpose() * Vj  + R_Ij * accel_body;
    dot_vqj = R_Ij * accel_body;
    /* publish r_j and v_j  */
    float sq_r = r_j(0)*r_j(0) + r_j(1)*r_j(1);

    if (Cable_Length_sq - sq_r>0.01)
    {
        B_j(2,0) =  -r_j(0)/sqrt((Cable_Length_sq - sq_r));
        B_j(2,1) =  -r_j(1)/sqrt((Cable_Length_sq - sq_r));
    } else {
        B_j(2,0) = -0.1;
        B_j(2,1) = -0.1;
    }
    BB_temp = B_j.transpose()*B_j; 
    float BB_determinent = BB_temp(0,0)*BB_temp(1,1) - BB_temp(0,1)*BB_temp(1,0);
    BB_inverse(0,0) = BB_temp(1,1);
    BB_inverse(1,1) = BB_temp(0,0);
    BB_inverse(0,1) = - BB_temp(0,1);
    BB_inverse(1,0) = - BB_temp(1,0);
    BB_inverse = BB_inverse/BB_determinent;
    BB_j = B_j * BB_inverse * B_j.transpose();

    /*Step 3 calculate payload position and attitude error*/
    pos_error = Xp-Xpd;
    float scale_p = sqrt(1+ pos_error.transpose() * pos_error);
    pos_error = pos_error/scale_p;

    angle_error = 0.5* Veemap(R_IPd.transpose()*R_IP- R_IP.transpose() * R_IPd);

    /* put a hard constraint on the angle_error */
    
    /*Step 4 calculate control force  control law form the GNC 2019 paper*/
    Eigen::Vector3f U = Vp + kv*pos_error - R_IP * TetherOffsetCross*(Omega_p + kR*angle_error) + B_j*(v_j + kL*r_j);
    for (int i = 0; i < 3; i++)
    {
        U(i) =  constrain_function(U(i), fp_max(i));
        u_d(i) = 0;
        // additional feedback based on payload relative position:
    }
    u_l = - Kphi * (Vj+kv*pos_error -  kR * R_IP * TetherOffsetCross * angle_error + kL*B_j*r_j);

    /*Step 5 calculate Delta_j_hat based on quadrotor acceleration feedback */

    // the estimation is activated only if the quadrotor is in off-board mode and reset the estimation to zero
    for (int i= 0; i < 3; i++) {
        f_L_j(i) = (float)throttle_sp(i);
    }
    if(_DroneState.mode != "OFFBOARD") {
        Delta_j<< 0.0,
                  0.0,
                  0.0; 
    } else {
        Delta_j +=  lambda_j * (Quad_MASS * dot_vqj - f_L_j - Delta_j);
    }
    // constraint the pose
    Delta_j_p = (Identity - L_j*L_j.transpose()/Cable_Length_sq)*Delta_j;
    Delta_j_p =  constrain_vector(Delta_j_p, 5.0);

    Delta_pt(0) = _AddonForce.delta_Tx;
    Delta_pt(1) = _AddonForce.delta_Ty;
    Delta_pt(2) = _AddonForce.delta_Tz;
    Delta_rt(0) = _AddonForce.delta_Rx;
    Delta_rt(1) = _AddonForce.delta_Ry;
    Delta_rt(2) = _AddonForce.delta_Rz;
   
    /*TO DO use intergral to facilitate robust control*/
    /* u_d is the estimated external disturbance*/

    u_d = (Delta_j_p + PayloadSharingPortion * Delta_pt + R_IP * E_j * Delta_rt)/TotalLiftedMass;

    // desired acceleration
    accel_sp[0] = u_l[0] - u_d[0];
    accel_sp[1] = u_l[1] - u_d[1];
    accel_sp[2] = u_l[2] - u_d[2] + 9.8;
    
    // desired thrust  = desired accel
    thrust_sp   =  px4_command_utils::accelToThrust(accel_sp, TotalLiftedMass, tilt_max);
    throttle_sp =  px4_command_utils::thrustToThrottle(thrust_sp);

    for (int i=0; i<3; i++)
    {
        _ControlOutput.u_l[i] = u_l[i];
        _ControlOutput.u_d[i] = u_d[i];
        _ControlOutput.Thrust[i]   = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }
     // publish fleet status
    _FleetStatus.r_jx = (double)r_j(0);
    _FleetStatus.r_jy = (double)r_j(1);
    _FleetStatus.v_jx = (double)v_j(0);
    _FleetStatus.v_jy = (double)v_j(1);
    _FleetStatus.f_Ljx = throttle_sp[0];
    _FleetStatus.f_Ljy = throttle_sp[1];
    _FleetStatus.f_Ljz = throttle_sp[2];
    _FleetStatus.delta_jx = (double)Delta_j_p(0);
    _FleetStatus.delta_jy = (double)Delta_j_p(1);
    _FleetStatus.delta_jz = (double)Delta_j_p(2);
    // publish fleet status
    Sendfleetstate();
    return _ControlOutput;

}
void payload_controller_TCST::Sendfleetstate()
{
    /*publish control force and quadrotor relative position*/
    pubFleetState.publish(_FleetStatus);
}
void payload_controller_TCST::GetAddonForce(const px4_command::AddonForce::ConstPtr &msg)
{
    _AddonForce = *msg;
}
void payload_controller_TCST::printf_result()
{
    cout <<">>>>>>>>>>  TCST 2019 Paylaod Pose Controller  <<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
    // use roserive to transport data to the ground station

    cout <<">>>>>>>> Payload control method in TCST 2019 paper (Parameter)  <<<<<" <<endl;
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
