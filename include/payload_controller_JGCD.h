#ifndef PAYLOAD_CONTROLLER_JGCD_H
#define PAYLOAD_CONTROLLER_JGCD_H

/***************************************************************************************************************************
*Author: Longhao Qian
*Date : 2020 08 01

* Linear + MPC control from JGCD paper
* can perform rectangular tracking action
* the estimator for disturbances on each drone is the same as the one in TCST paper
***************************************************************************************************************************/

#include <Eigen/Eigen>
#include <math.h>
#include <iostream>
#include <string>
#include <command_to_mavros.h>
#include <px4_command_utils.h>
#include <math_utils.h>
#include <rectangular_trajectory.h>
#include <quadrotor_drone.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/ControlOutput.h>
#include <px4_command/ControlParameter.h>
#include <px4_command/AuxiliaryState.h>
#include <px4_command/Emergency.h>
#include <px4_command/FleetStatus.h>
#include <px4_command/AddonForce.h>
using std::string;
using std::iostream;
namespace multidronepayload {
    class payload_controller_JGCD
    {
        public:
            payload_controller_JGCD(char drone_ID[20],ros::NodeHandle& main_handle);
            ~payload_controller_JGCD();
            // topic setup
            void ros_topic_setup(ros::NodeHandle& main_handle);
            //Printf the controller parameter
            void printf_param();
            void printf_result();
            //Determine whether the system should enter an emergency landing
            bool emergency_switch();
            // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference; 
            px4_command::ControlOutput payload_controller(const px4_command::DroneState&      _DroneState, 
                                                          const px4_command::TrajectoryPoint& _Reference_State, 
                                                          float dt);                                            
            // control command for main program
            px4_command::ControlOutput _ControlOutput;
        private:
            // ------------  functions-----------------//
            void GetAddonForce(const px4_command::AddonForce::ConstPtr& msg);
            void SendFleetStatus();
            void pubauxiliarystate();
            void send_parameter_to_ground_station();
            Eigen::Matrix3f CalculateAuxiliaryE(const Eigen::Matrix3f& R);
            //------------- variables ---------------//
            ros::ServiceClient            clientSendParameter;
            px4_command::ControlParameter ParamSrv;
            ros::Publisher                pubAuxiliaryState;
            ros::Publisher                pubFleetStatus;
            ros::Subscriber               subAddonForce;
            // logic states:
            bool   isPubAuxiliaryState;
            bool   isEmergency;
            bool   isEmergencyFromLeader;
            bool   isPerformingAction;
            // action info
            int type;
            trajectory::Reference_Path rect_path;
            trajectory::Rectangular_Trajectory_Parameter rect_param;
            trajectory::Rectangular_Trajectory rec_traj;
            // ggeo_fence
            Eigen::Vector2f geo_fence_x;
            Eigen::Vector2f geo_fence_y;
            Eigen::Vector2f geo_fence_z;
            // drone parameter:
            string uav_pref;
            experiment_drone::quadrotor_parameter drone_parameter;
            experiment_drone::quadrotor_drone drone;
            /* payload configuration parameters*/
            int   num_drone;
            float Payload_Mass;
            float TotalLiftedMass;
            float Cable_Length;
            float Cable_Length_sq;
            float Cable_Tolerance;
            float MaximumInclination;
            Eigen::Vector3f TetherOffset;
            Eigen::Matrix3f TetherOffsetCross;
            float  PayloadSharingPortion;
            Eigen::Matrix3f J_p;
            px4_command:: AuxiliaryState Auxstate;
            //Controller parameter for the control law
            Eigen::Matrix3f kv;
            Eigen::Matrix3f kR;
            Eigen::Matrix3f Kphi;
            float kL;
            Eigen::Matrix3f Ej;
            Eigen::Matrix3f D;
            // command 
            Eigen::Vector3d    AttitudeTargetEuler;
            Eigen::Quaterniond AttitudeTargetQuaterniond;
            Eigen::Vector4f AttitudeTargetQuaternionv;
            Eigen::Vector4f AttitudeQuaternionv;
            Eigen::Matrix3f R_IPd;
            // disturbance estimation result:
            Eigen::Vector3f PayloadDisturbance;
            Eigen::Vector3f Delta_pt;
            Eigen::Vector3f Delta_rt;
            Eigen::Matrix3f lambda_j;
            Eigen::Matrix2f BB_temp; 
            Eigen::Matrix2f BB_inverse;
            Eigen::Matrix3f BB_j;// estimator matrix
            Eigen::Vector3f Delta_j;
            Eigen::Vector3f Delta_j_p;
            // payload state
            Eigen::Vector3f Xp, Vp, Xj, Vj, Omega_p, Xpd;
            Eigen::Matrix3f R_IP;
            Eigen::Vector3f L_j_dot;//rotation speed of the cable tip
            Eigen::Vector3f L_j;    // the cable vector
            Eigen::Vector2f r_j, v_j;
            Eigen::Matrix<float, 3,2> B_j, B_j_dot;
            // virtural control force
            Eigen::Vector3f accel_sp;
            Eigen::Vector3f f_L_j;
            Eigen::Vector3f f_p_j;
            Eigen::Vector3f U;
            Eigen::Vector3f u_l;
            Eigen::Vector3f u_d;
            // additive MPC term from leader drone 
            Eigen::Vector2f rd_j, mu_j;
            Eigen::Vector3f f_pj, f0_j, f_bj, f_cj;// trim force and motion sync term
            Eigen::Vector3f mpc_T, mpc_R; // additive mpc term for payload translation and rotation  
            // error constraint parameters:
            Eigen::Vector3f pos_error;
            Eigen::Vector3f vel_error;
            Eigen::Vector3f angle_error;
            Eigen::Vector3f pos_error_max;
            Eigen::Vector3f angular_error_max;
            Eigen::Vector3f s_P;
            Eigen::Vector3f s_R;
            Eigen::Vector3f fp_max;
            // some constants
            Eigen::Matrix3f Identity;
            Eigen::Matrix3f Identity_30;
            Eigen::Matrix2f Identity2D;
            Eigen::Vector3f g_I;
            Eigen::Vector3f e_3;
    };

    payload_controller_JGCD::payload_controller_JGCD(char drone_ID[20],ros::NodeHandle& main_handle){
        // use drone ID to get the correct name for parameters
        uav_pref = "uav";
        uav_pref = uav_pref + drone_ID[0];
        drone_parameter.uav_name = uav_pref;
        // load uav and payload parameters
        main_handle.param<float>(uav_pref + "_Pos_GNC/mass", drone_parameter.Quad_MASS, 1.0);
        main_handle.param<float>(uav_pref + "_Pos_GNC/cablelength", Cable_Length, 1.0);
        main_handle.param<int>("Pos_JGCD/num_drone",num_drone,1);
        main_handle.param<float>(uav_pref + "_Pos_GNC/TetherOffset_x", TetherOffset(math_utils::Vector_X), 0.5);
        main_handle.param<float>(uav_pref + "_Pos_GNC/TetherOffset_y", TetherOffset(math_utils::Vector_Y), 0);
        main_handle.param<float>(uav_pref + "_Pos_GNC/TetherOffset_z", TetherOffset(math_utils::Vector_Z), 0);
        main_handle.param<double>(uav_pref + "_Pos_GNC/motor_slope",     drone_parameter.liftmodel.motor_slope,0.3);
        main_handle.param<double>(uav_pref + "_Pos_GNC/motor_intercept", drone_parameter.liftmodel.motor_intercept, 0.0);
        main_handle.param<float>(uav_pref + "_Pos_GNC/PayloadSharingPortion", PayloadSharingPortion, 0.5);
        main_handle.param<float>("Payload/mass", Payload_Mass, 1.0);
        main_handle.param<bool>("Pos_JGCD/PubAuxiliaryState", isPubAuxiliaryState, true);
        // set the estimated payload moment of inertia
        J_p.setZero();
        J_p(0,0) = 0.01;
        J_p(1,1) = 0.01;
        J_p(2,2) = 0.1;
        TetherOffsetCross = Hatmap(TetherOffset);
        D.setZero();
        Eigen::Vector3f temp_offset;
        float e_n = 1.0;
        float a_j;
        for (int i = 0; i< num_drone; i++) {
            temp_offset.setZero();
            a_j = 0;
            main_handle.param<float>("uav" + to_string(i) + "_Pos_GNC/ayloadSharingPortion", a_j, 0.5);
            main_handle.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_x", temp_offset(math_utils::Vector_X), 0.5);
            main_handle.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_y", temp_offset(math_utils::Vector_Y), 0);
            main_handle.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_z", temp_offset(math_utils::Vector_Z), 0);
            D += a_j * Hatmap(temp_offset) * Hatmap(temp_offset);
            e_n *= temp_offset.norm();
        }
        if (num_drone < 3) {
            Ej = - TetherOffsetCross / e_n;
        } else {
            Ej = TetherOffsetCross * D.inverse(); // Ej = t_jx D^-1 if at least 3 drones are used.
        }
        Cable_Length_sq = Cable_Length * Cable_Length;
        TotalLiftedMass = Payload_Mass* PayloadSharingPortion + drone_parameter.Quad_MASS;
        kv.setZero();
        kR.setZero();
        
        main_handle.param<float>("Pos_JGCD/kv_xy", kv(math_utils::Vector_X,math_utils::Vector_X), 0.2);
        main_handle.param<float>("Pos_JGCD/kv_xy", kv(math_utils::Vector_Y,math_utils::Vector_Y), 0.2);
        main_handle.param<float>("Pos_JGCD/Kv_z",  kv(math_utils::Vector_Z,math_utils::Vector_Z), 0.4);

        main_handle.param<float>("Pos_JGCD/kR_xy" , kR(math_utils::Vector_X,math_utils::Vector_X), 0.2);
        main_handle.param<float>("Pos_JGCD/kR_xy" , kR(math_utils::Vector_Y,math_utils::Vector_Y), 0.2);
        main_handle.param<float>("Pos_JGCD/kR_z"  , kR(math_utils::Vector_Z,math_utils::Vector_Z), 0.4);

        main_handle.param<float>("Pos_JGCD/kL",  kL, 0.5);

        main_handle.param<float>("Limit/pxy_error_max", pos_error_max[math_utils::Vector_X], 0.6);
        main_handle.param<float>("Limit/pxy_error_max", pos_error_max[math_utils::Vector_Y], 0.6);
        main_handle.param<float>("Limit/pz_error_max" , pos_error_max[math_utils::Vector_Z], 1.0);

        main_handle.param<float>("Limit/tilt_max", drone_parameter.tiltlimit, 20.0);

    
        main_handle.param<float>("Pos_JGCD/fp_max_x", fp_max(math_utils::Vector_X),1);
        main_handle.param<float>("Pos_JGCD/fp_max_y", fp_max(math_utils::Vector_Y),1);
        main_handle.param<float>("Pos_JGCD/fp_max_z", fp_max(math_utils::Vector_Z),1);
    
        main_handle.param<float>("Pos_JGCD/MaxInclination", MaximumInclination , 40.0);
        main_handle.param<float>("Pos_JGCD/CableLengthTolerance", Cable_Tolerance, 1.2);

        lambda_j.setZero();

        main_handle.param<float>("Pos_JGCD/lambda_j", lambda_j(math_utils::Vector_X,math_utils::Vector_X), 1);
        main_handle.param<float>("Pos_JGCD/lambda_j", lambda_j(math_utils::Vector_Y,math_utils::Vector_Y), 1);
        main_handle.param<float>("Pos_JGCD/lambda_j", lambda_j(math_utils::Vector_Z,math_utils::Vector_Z), 1);

        Kphi.setZero();
        main_handle.param<float> ("Pos_JGCD/Kphi_xy", Kphi(math_utils::Vector_X,math_utils::Vector_X), 0.0);
        main_handle.param<float> ("Pos_JGCD/Kphi_xy", Kphi(math_utils::Vector_Y,math_utils::Vector_Y), 0.0);
        main_handle.param<float> ("Pos_JGCD/Kphi_z", Kphi(math_utils::Vector_Z,math_utils::Vector_Z), 0.0);        

        // special parameters
        Identity.setIdentity();
        Identity_30  << 1.0,0.0,0.0,
                        0.0,1.0,0.0,
                        0.0,0.0,0.0;
        Identity2D  <<  1.0,0.0,
                        0.0,1.0;       
        g_I<<0.0,
             0.0,
            -9.81;
        R_IP.setIdentity();
        R_IPd.setIdentity();
        B_j << 1.0,0.0,
               0.0,1.0,
               0.0,0.0;
        B_j_dot.setZero();
        Delta_pt.setZero();
        Delta_rt.setZero();
        Delta_j.setZero();
        Delta_j_p.setZero();
        vel_error.setZero();
        accel_sp.setZero();
        f_L_j.setZero();          
        u_l.setZero();
        u_d.setZero();
        r_j.setZero();
        v_j.setZero();
        rd_j.setZero();
        f_p_j.setZero();
        f_bj.setZero();
        f_cj.setZero();
        s_P.setZero();
        s_R.setZero();
        U.setZero();
        mpc_T.setZero();
        mpc_R.setZero();
        e_3<<0.0,
             0.0,
             1.0;
        PayloadDisturbance.setZero();
        isEmergencyFromLeader= false;
        isEmergency          = false; 
        isPerformingAction   = false;
        // load drone parameter
        drone.loadparameter(drone_parameter);
        // read the geo geo_fence
        main_handle.param<float>("payload_geofence/x_min", geo_fence_x[math_utils::Vector_X], -0.6);
        main_handle.param<float>("payload_geofence/x_max", geo_fence_x[math_utils::Vector_Y], 0.6);
        main_handle.param<float>("payload_geofence/y_min", geo_fence_y[math_utils::Vector_X], -0.3);
        main_handle.param<float>("payload_geofence/y_max", geo_fence_y[math_utils::Vector_Y], 0.3);
        main_handle.param<float>("payload_geofence/z_min", geo_fence_z[math_utils::Vector_X],-0.05);
        main_handle.param<float>("payload_geofence/z_max", geo_fence_z[math_utils::Vector_Y], 0.6);
        // read the trajectory information
        main_handle.param<int>("ActionMode/type", type, 1);
        switch (type) {
              case 1:
              {
                main_handle.param<float>("Rectangular_Trajectory/a_x",   rect_param.a_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/a_y",   rect_param.a_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/vel_x", rect_param.v_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/vel_y", rect_param.v_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/h",     rect_param.h, 0.0);

                main_handle.param<float>("Rectangular_Trajectory/center_x",     rect_param.center_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_y",     rect_param.center_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_z",     rect_param.center_z, 0.0);

                rec_traj.LoadParameter(rect_param);
                break;
              }
              default:
              {
                main_handle.param<float>("Rectangular_Trajectory/a_x",   rect_param.a_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/a_y",   rect_param.a_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/vel_x", rect_param.v_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/vel_y", rect_param.v_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/h",     rect_param.h, 0.0);

                main_handle.param<float>("Rectangular_Trajectory/center_x",     rect_param.center_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_y",     rect_param.center_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_z",     rect_param.center_z, 0.0);

                rec_traj.LoadParameter(rect_param);
                break;
              }
        }
        
    }

    payload_controller_JGCD::~payload_controller_JGCD(){

    }


    void payload_controller_JGCD::ros_topic_setup(ros::NodeHandle& main_handle){
        // setting up communication channel:
        cout << " ros topic setup! (JGCD) " <<endl;
        if (isPubAuxiliaryState)  {
            pubAuxiliaryState   = main_handle.advertise<px4_command::AuxiliaryState > ("/" + uav_pref + "/px4_command/auxiliarystate", 1000);
        }
        clientSendParameter =  main_handle.serviceClient<px4_command::ControlParameter>("/" + uav_pref + "/px4_command/parameters");
        pubFleetStatus      =  main_handle.advertise<px4_command::FleetStatus>("/" + uav_pref + "/px4_command/fleetstatus", 1000);
        subAddonForce       =  main_handle.subscribe<px4_command::AddonForce>("/uav0/px4_command/addonforce", 100, &payload_controller_JGCD::GetAddonForce, this);
    }


    px4_command::ControlOutput payload_controller_JGCD::payload_controller(
        const px4_command::DroneState& _DroneState, 
        const px4_command::TrajectoryPoint& _Reference_State, 
        float dt) 
    {
        /* step 1: get command attitude and payload attitude*/
        AttitudeTargetEuler(0) = (double)_Reference_State.roll_ref;
        AttitudeTargetEuler(1) = (double)_Reference_State.pitch_ref;
        AttitudeTargetEuler(2) = (double)_Reference_State.yaw_ref;
        AttitudeTargetQuaterniond = quaternion_from_rpy(AttitudeTargetEuler);
        AttitudeTargetQuaternionv(0) = (float)AttitudeTargetQuaterniond.w();
        AttitudeTargetQuaternionv(1) = (float)AttitudeTargetQuaterniond.x();
        AttitudeTargetQuaternionv(2) = (float)AttitudeTargetQuaterniond.y();
        AttitudeTargetQuaternionv(3) = (float)AttitudeTargetQuaterniond.z();
        R_IPd =  QuaterionToRotationMatrix(AttitudeTargetQuaternionv);
        // get the payload rotation matrix
        for(int i = 0; i < 4 ;i++){
            AttitudeQuaternionv(i) = _DroneState.payload_quaternion[i];
        }

        R_IP = QuaterionToRotationMatrix(AttitudeQuaternionv);
        // update drone state
        drone.updatestate(_DroneState);
        /*------Step 2 calculate L_j and L_j_dot based on payload information feedback -------*/
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

        B_j_dot.row(2) = - v_j.transpose() * ((Cable_Length_sq - sq_r)*Identity2D 
                        + r_j * r_j.transpose()) / pow(sqrt((Cable_Length_sq - sq_r)),3);

        BB_temp = B_j.transpose()*B_j; 
        float BB_determinent = BB_temp(0,0)*BB_temp(1,1) - BB_temp(0,1)*BB_temp(1,0);
        BB_inverse(0,0) = BB_temp(1,1);
        BB_inverse(1,1) = BB_temp(0,0);
        BB_inverse(0,1) = - BB_temp(0,1);
        BB_inverse(1,0) = - BB_temp(1,0);
        BB_inverse = BB_inverse/BB_determinent;
        BB_j = B_j * BB_inverse * B_j.transpose();

        /*------Step 3 calculate payload position and attitude error--------*/
        /*  a. position error:
        determine whether the system is performing action */
        
        if(isPerformingAction){
            switch (type)
            {
                case 1:{
                    rect_path = rec_traj.UpdatePosition(Xp);
                    vel_error = Vp - rect_path.vd*rect_path.n;
                    pos_error = (Identity - rect_path.n*rect_path.n.transpose())*(Xp - rect_path.P);
                    break;
                }
                default:{
                    rect_path = rec_traj.UpdatePosition(Xp);
                    vel_error = Vp - rect_path.vd*rect_path.n;
                    pos_error = (Identity - rect_path.n*rect_path.n.transpose())*(Xp - rect_path.P);
                break;
                }
            }
        }else{
            pos_error = Xp - Xpd;
            vel_error = Vp; // position stabilization
        }

        float scale_p = sqrt(1 + pos_error.transpose() * pos_error);
        pos_error = pos_error/scale_p; 
        s_P = vel_error + kv * pos_error;

        // b. angular error:
        angle_error = 0.5* Veemap(R_IPd.transpose()*R_IP- R_IP.transpose() * R_IPd);
        if (num_drone<3) {
            angle_error(math_utils::Vector_X) = 0.0; // if only two drones are involved, we have to remove one axis in control
        }
        s_R = Omega_p + kR  * angle_error;
        /*------- Step 4 calculate control law form the GNC 2019 paper ---------*/
        // a. calculate disturbance force on the quadrotor
        if(_DroneState.mode != "OFFBOARD") {
            Delta_j.setZero(); // estimation does not run in offboard mode
        } else {
            Delta_j +=  lambda_j * dt * BB_j * (drone_parameter.Quad_MASS * (drone.getProcessedIMU().AccInertial- g_I) - f_L_j - Delta_j);
        }
        Delta_j_p = (Identity - L_j*L_j.transpose()/Cable_Length_sq)*Delta_j; // get the effective disturbance
        Delta_j_p =  constrain_vector(Delta_j_p, 10.0);// constraint the disturbance estimation.
        // b. calculate the payload compensation force:
        f_p_j = - PayloadSharingPortion * (Payload_Mass*g_I + Delta_pt + R_IP * Ej * Delta_rt);
        // calculate corresponding desired cable tilt angle:
        rd_j = Cable_Length *  f_p_j.segment<2>(0)/f_p_j.norm(); 
        // c. calculate mu_j term in the paper
        PayloadDisturbance =  PayloadSharingPortion * ( Delta_pt + R_IP * Ej * Delta_rt);
        mu_j = kL* (r_j - rd_j); 
        // d. calculate total control force:
        U = (s_P + mpc_T) - R_IP * TetherOffsetCross*(s_R + mpc_R) + B_j*(v_j + mu_j);  
        u_l = - Kphi * U - (Delta_j_p + PayloadDisturbance)/TotalLiftedMass;
        /*------- Step 5 pass the calculated control force to FCU ---------*/
        accel_sp[math_utils::Vector_X] = u_l[math_utils::Vector_X] - u_d[math_utils::Vector_X];
        accel_sp[math_utils::Vector_Y] = u_l[math_utils::Vector_Y] - u_d[math_utils::Vector_Y];
        accel_sp[math_utils::Vector_Z] = u_l[math_utils::Vector_Z] - u_d[math_utils::Vector_Z] + 9.81;
        Eigen::Vector3f temp_F = TotalLiftedMass * accel_sp;
        f_L_j = temp_F.norm() * (drone.getProcessedIMU().R_Ij * e_3);
        // update the required thrust
        _ControlOutput = drone.outputdronecommand(accel_sp, TotalLiftedMass, u_l, u_d);
        SendFleetStatus();// then send the fleet status for robust control
        if (isPubAuxiliaryState) {
            pubauxiliarystate();// send the auxiliary status for analysis
        }       
        return _ControlOutput;

    }

    Eigen::Matrix3f payload_controller_JGCD::CalculateAuxiliaryE(const Eigen::Matrix3f& R) {

        return 0.5*(R.trace()*Identity - R);

    }

    void payload_controller_JGCD::pubauxiliarystate(){
            // record time
            Auxstate.header.stamp = ros::Time::now();

            Auxstate.L_measured = L_j.norm();

            Auxstate.q_0 = drone.getProcessedIMU().Quad_Drone(math_utils::Quat_w);
            Auxstate.q_1 = drone.getProcessedIMU().Quad_Drone(math_utils::Quat_x);
            Auxstate.q_2 = drone.getProcessedIMU().Quad_Drone(math_utils::Quat_y);
            Auxstate.q_3 = drone.getProcessedIMU().Quad_Drone(math_utils::Quat_z);

            Auxstate.r_jx = r_j(math_utils::Vector_X);
            Auxstate.r_jy = r_j(math_utils::Vector_Y);

            Auxstate.v_jx = v_j(math_utils::Vector_X);
            Auxstate.v_jy = v_j(math_utils::Vector_Y);

            Auxstate.pos_error_x = pos_error(math_utils::Vector_X);
            Auxstate.pos_error_y = pos_error(math_utils::Vector_Y);
            Auxstate.pos_error_z = pos_error(math_utils::Vector_Z);

            Auxstate.angle_error_x =  angle_error(math_utils::Vector_X);
            Auxstate.angle_error_y =  angle_error(math_utils::Vector_Y);
            Auxstate.angle_error_z =  angle_error(math_utils::Vector_Z);
        
            Eigen::Vector3d Euler = quaternion_to_euler2(AttitudeQuaternionv);
            Auxstate.Euler_roll =   Euler(0)*57.3;
            Auxstate.Euler_pitch =  Euler(1)*57.3;
            Auxstate.Euler_yaw  =   Euler(2)*57.3;

            Auxstate.fLj_x  = f_L_j(math_utils::Vector_X);
            Auxstate.fLj_y  = f_L_j(math_utils::Vector_Y);
            Auxstate.fLj_z  = f_L_j(math_utils::Vector_Z);

            Auxstate.Delta_jp_x = Delta_j_p(math_utils::Vector_X);
            Auxstate.Delta_jp_y = Delta_j_p(math_utils::Vector_Y);
            Auxstate.Delta_jp_z = Delta_j_p(math_utils::Vector_Z);

            Auxstate.acc_x = drone.getProcessedIMU().AccInertial(math_utils::Vector_X);
            Auxstate.acc_y = drone.getProcessedIMU().AccInertial(math_utils::Vector_Y);
            Auxstate.acc_z = drone.getProcessedIMU().AccInertial(math_utils::Vector_Z);

            Auxstate.rd_jx = rd_j(math_utils::Vector_X);
            Auxstate.rd_jy = rd_j(math_utils::Vector_Y);

            pubAuxiliaryState.publish(Auxstate);
    }

    void payload_controller_JGCD::SendFleetStatus()
    {
        px4_command::FleetStatus FleetStatus_;
        FleetStatus_.header.stamp = ros::Time::now();
        FleetStatus_.r_jx = r_j(math_utils::Vector_X);
        FleetStatus_.r_jy = r_j(math_utils::Vector_Y);
        FleetStatus_.v_jx = v_j(math_utils::Vector_X);  
        FleetStatus_.v_jy = v_j(math_utils::Vector_Y);

        FleetStatus_.f_Ljx = 4.0*drone.getDroneCommand().thrustSetpoint(math_utils::Vector_X);
        FleetStatus_.f_Ljy = 4.0*drone.getDroneCommand().thrustSetpoint(math_utils::Vector_Y);       
        FleetStatus_.f_Ljz = 4.0*drone.getDroneCommand().thrustSetpoint(math_utils::Vector_Z);       

        FleetStatus_.delta_jx = Delta_j_p(math_utils::Vector_X);
        FleetStatus_.delta_jy = Delta_j_p(math_utils::Vector_Y);   
        FleetStatus_.delta_jz = Delta_j_p(math_utils::Vector_Z);   

        FleetStatus_.rd_jx = rd_j(math_utils::Vector_X);
        FleetStatus_.rd_jy = rd_j(math_utils::Vector_Y);

        FleetStatus_.emergency = isEmergency;
        pubFleetStatus.publish(FleetStatus_);
    }

    void payload_controller_JGCD::GetAddonForce(const px4_command::AddonForce::ConstPtr& msg)  
    {
        Delta_pt(math_utils::Vector_X) = (*msg).delta_Tx;
        Delta_pt(math_utils::Vector_Y) = (*msg).delta_Ty;
        Delta_pt(math_utils::Vector_Z) = (*msg).delta_Tz;
        Delta_rt(math_utils::Vector_X) = (*msg).delta_Rx;
        Delta_rt(math_utils::Vector_Y) = (*msg).delta_Ry;
        Delta_rt(math_utils::Vector_Z) = (*msg).delta_Rz;
        mpc_T(math_utils::Vector_X) = (*msg).R_1x;
        mpc_T(math_utils::Vector_Y) = (*msg).R_1y;
        mpc_T(math_utils::Vector_Z) = (*msg).R_1z;
        mpc_R(math_utils::Vector_X) = (*msg).R_2x;
        mpc_R(math_utils::Vector_Y) = (*msg).R_2y;  
        mpc_R(math_utils::Vector_Z) = (*msg).R_2z; 
        isPerformingAction = (*msg).perform_action;
        isEmergencyFromLeader = (*msg).emergency;
    }

    bool payload_controller_JGCD::emergency_switch()
    {
        
        /* 1. check whether the total length of the string is within a safe range */
        if (L_j.norm()>Cable_Length*Cable_Tolerance)
        {
            isEmergency = true;
        }
        /* 2. check if the r_j is too large */

        if (r_j.norm()>Cable_Length*sin(MaximumInclination/57.3))
        {
            isEmergency = true;
        }

        /* 3. check if the emergency signal from the leader drone is true*/

        return isEmergency;   
    }

    void payload_controller_JGCD::send_parameter_to_ground_station()
    {
        
        ParamSrv.request.controllername      = uav_pref + " with JGCD";
        ParamSrv.request.dronemass           = drone_parameter.Quad_MASS;
        ParamSrv.request.cablelength         = Cable_Length;
        ParamSrv.request.a_j                 = PayloadSharingPortion;
        ParamSrv.request.payloadmass = Payload_Mass;
        ParamSrv.request.num_drone   = num_drone;
        ParamSrv.request.motor_slope = (float)drone_parameter.liftmodel.motor_slope;
        ParamSrv.request.motor_intercept = (float)drone_parameter.liftmodel.motor_intercept;
        ParamSrv.request.isPubAuxiliaryState     = isPubAuxiliaryState;
        ParamSrv.request.isAddonForcedUsed       = true;
        ParamSrv.request.isCrossFeedingTermsUsed = false;
        ParamSrv.request.t_jx        = TetherOffset(math_utils::Vector_X);
        ParamSrv.request.t_jy        = TetherOffset(math_utils::Vector_Y);
        ParamSrv.request.t_jz        = TetherOffset(math_utils::Vector_Z);
        ParamSrv.request.kv_xy       = kv(math_utils::Vector_X,math_utils::Vector_X);
        ParamSrv.request.kv_z        = kv(math_utils::Vector_Z,math_utils::Vector_Z);
        ParamSrv.request.kr1_x       = 0.0;
        ParamSrv.request.kr1_y       = 0.0;
        ParamSrv.request.kr1_z       = 0.0;
        ParamSrv.request.kr2_x       = 0.0;
        ParamSrv.request.kr2_y       = 0.0;
        ParamSrv.request.kr2_z       = 0.0;
        ParamSrv.request.kp_x        = 0.0;
        ParamSrv.request.kp_y        = 0.0;
        ParamSrv.request.kp_z        = 0.0;
        ParamSrv.request.komega_x    = 0.0;
        ParamSrv.request.komega_y    = 0.0;
        ParamSrv.request.komega_z    = 0.0;
        ParamSrv.request.lambdaj_x   = lambda_j(math_utils::Vector_X,math_utils::Vector_X);
        ParamSrv.request.lambdaj_y   = lambda_j(math_utils::Vector_Y,math_utils::Vector_Y);
        ParamSrv.request.lambdaj_z   = lambda_j(math_utils::Vector_Z,math_utils::Vector_Z);
        if (uav_pref.compare("uav0")==0) {
            
        ros::NodeHandle main_handle("~");
        main_handle.param<float> ("Pos_JGCD/lambda_Txy", ParamSrv.request.lambda_T_x, 0.0);
        main_handle.param<float> ("Pos_JGCD/lambda_Txy", ParamSrv.request.lambda_T_y, 0.0);
        main_handle.param<float> ("Pos_JGCD/lambda_Tz",  ParamSrv.request.lambda_T_z, 0.0);
        main_handle.param<float> ("Pos_JGCD/lambda_Rxy", ParamSrv.request.lambda_R_x, 0.0);
        main_handle.param<float> ("Pos_JGCD/lambda_Rxy", ParamSrv.request.lambda_R_y, 0.0);
        main_handle.param<float> ("Pos_JGCD/lambda_Rz",  ParamSrv.request.lambda_R_z, 0.0);}

        ParamSrv.request.lambda1_x   = 0.0;
        ParamSrv.request.lambda1_y   = 0.0;
        ParamSrv.request.lambda1_z   = 0.0;
        ParamSrv.request.lambda2_x   = 0.0;
        ParamSrv.request.lambda2_y   = 0.0;
        ParamSrv.request.lambda2_z   = 0.0;
        ParamSrv.request.kR_xy           = kR(math_utils::Vector_X,math_utils::Vector_X);
        ParamSrv.request.kR_z            = kR(math_utils::Vector_Z,math_utils::Vector_Z);
        ParamSrv.request.kL              = kL;
        ParamSrv.request.Kphi_xy         = Kphi(math_utils::Vector_X,math_utils::Vector_X);
        ParamSrv.request.Kphi_z          = Kphi(math_utils::Vector_Z,math_utils::Vector_Z);
        ParamSrv.request.pxy_error_max   = pos_error_max[math_utils::Vector_X];
        ParamSrv.request.pz_error_max    = pos_error_max[math_utils::Vector_Z];
        ParamSrv.request.pxy_int_max     = 0.0;
        ParamSrv.request.pz_int_max      = 0.0;
        ParamSrv.request.tilt_max        = drone_parameter.tiltlimit;
        ParamSrv.request.int_start_error = 0.0;
        ParamSrv.request.fp_max_x        = fp_max(math_utils::Vector_X);
        ParamSrv.request.fp_max_y        = fp_max(math_utils::Vector_Y);
        ParamSrv.request.fp_max_z        = fp_max(math_utils::Vector_Z);
        // call the ground station to recieve the parameters and waiting for connection....
        bool isresponserecieved = false;
        ros::Time begin_time    = ros::Time::now();
        float last_time         = px4_command_utils::get_time_in_sec(begin_time);
        float cur_time          = last_time;
        while (!isresponserecieved) {
            // very 3 seconds, send a parameter service call to the ground station.
            cur_time = px4_command_utils::get_time_in_sec(begin_time);
            if(((int)(cur_time*10) % 30) == 0) {
                ROS_INFO("Waiting for response from ground station..., time elapsed %f [s]",cur_time);  
                isresponserecieved = clientSendParameter.call(ParamSrv);
            }
        }
        ROS_INFO("Parameter sent to ground station !");          
    }

    void payload_controller_JGCD::printf_result()
    {
        cout <<">>>>>>>>  JGCD 2020 Paylaod Pose Controller "<< uav_pref <<" <<<<<<<" <<endl;

        //固定的浮点显示
        cout.setf(ios::fixed);
        //左对齐
        cout.setf(ios::left);
        // 强制显示小数点
        cout.setf(ios::showpoint);
        // 强制显示符号
        cout.setf(ios::showpos);

        cout<<setprecision(3);

        if (!isEmergency) {
            cout << ">>>>>>  Payload Measurements and Errors <<<<<<<<<" <<endl;
            // priint out the control status.
            cout << "r_j [X Y] : " << r_j(math_utils::Vector_X) << " [m] " << r_j(math_utils::Vector_Y) << " [m] \n";
            cout << "v_j [X Y] : " << v_j(math_utils::Vector_X) << " [m/s] "<< v_j(math_utils::Vector_Y) << " [m/s] \n";
            cout << "length from mocap: " << L_j.norm() << " [m] " <<endl;
            cout << "Pos Error [X Y Z] : " << pos_error[math_utils::Vector_X] << " [m] " 
                                           << pos_error[math_utils::Vector_Y] << " [m] " 
                                           << pos_error[math_utils::Vector_Z] << " [m] \n";
                                        
            cout << "Angular Error [X Y Z] : " << angle_error[math_utils::Vector_X] << " [] "
                                               << angle_error[math_utils::Vector_Y] << " [] " 
                                               << angle_error[math_utils::Vector_Z] << " [] \n";

            cout << ">>>>>> Payload Target Attitude and Attitude Verification <<<<<<<<<" <<endl;
            // verify the target quaternion has been calculated:
            Eigen::Vector3d Euler_Target = quaternion_to_euler2(AttitudeTargetQuaternionv);
            cout << "Target Euler, roll : "  << Euler_Target(0)*57.3 << " [DEG] ";
            cout << "pitch : " << Euler_Target(1)*57.3 << " [DEG] ";
            cout << "yaw : "   << Euler_Target(2)*57.3 << " [DEG] ";
            cout << endl;
            // display rotation matrix:
            cout << "R_IP: " <<endl;
            cout << R_IP<<endl;
            cout << "R_IPd: " <<endl;
            cout << R_IPd<<endl;
            // verify that the payload attitude
            Eigen::Vector3d Euler = quaternion_to_euler2(AttitudeQuaternionv);
            cout << "Current Euler, roll : "  << Euler(0)*57.3 << " [DEG] ";
            cout << "pitch : " << Euler(1)*57.3 << " [DEG] ";
            cout << "yaw : "   << Euler(2)*57.3 << " [DEG] ";
            cout << endl;
            cout << ">>>>>> MPC control <<<<<<<<<" <<endl;
            cout << "MPC_T x: "<< mpc_T(math_utils::Vector_X) << " [N] ";
            cout << "MPC_T y: "<< mpc_T(math_utils::Vector_Y) << " [N] ";
            cout << "MPC_T z: "<< mpc_T(math_utils::Vector_Z) << " [N] ";
            cout << endl;
            cout << "MPC_R x: "<< mpc_R(math_utils::Vector_X) << " [N] ";
            cout << "MPC_R y: "<< mpc_R(math_utils::Vector_Y) << " [N] ";
            cout << "MPC_R z: "<< mpc_R(math_utils::Vector_Z) << " [N] ";
            cout << endl;             

            cout << ">>>>>> disturbance estimation <<<<<<<<<" <<endl;
            // display estimation force on quadrotor
            cout << "Delta_jp x: " << Delta_j_p (math_utils::Vector_X)<< " [N] ";
            cout << "Delta_jp y: " << Delta_j_p (math_utils::Vector_Y)<< " [N] ";
            cout << "Delta_jp z: " << Delta_j_p (math_utils::Vector_Z)<< " [N] ";
            cout << endl;
            /* display addon on force*/
            cout << "Delta_pt x: "<< Delta_pt(math_utils::Vector_X) << " [N] ";
            cout << "Delta_pt y: "<< Delta_pt(math_utils::Vector_Y) << " [N] ";
            cout << "Delta_pt z: "<< Delta_pt(math_utils::Vector_Z) << " [N] ";
            cout << endl;
            cout << "Delta_rt x: "<< Delta_rt(math_utils::Vector_X) << " [N] ";
            cout << "Delta_rt y: "<< Delta_rt(math_utils::Vector_Y) << " [N] ";
            cout << "Delta_rt z: "<< Delta_rt(math_utils::Vector_Z) << " [N] ";
            cout << endl;   
            /* display the total compensation force for payload*/ 
            cout << "f_p_j [X Y Z]: " << f_p_j(math_utils::Vector_X)<< " [N] " 
                                      << f_p_j(math_utils::Vector_Y)<< " [N] " 
                                      << f_p_j(math_utils::Vector_Z)<< " [N] \n";
            /* acc setpoint */
            cout << " accel setpoint [X Y Z]: "<< accel_sp(math_utils::Vector_X)<< " [m/s^2] " 
                                      << accel_sp(math_utils::Vector_Y)<< " [m/s^2] " 
                                      << accel_sp(math_utils::Vector_Z)<< " [m/s^2] \n";
            /* display the desired cable inclination */
            cout << "rd_j : " << rd_j(math_utils::Vector_X) << " [m] " << rd_j(math_utils::Vector_Y) << " [m] " <<endl;
            /* display action */
            if(isPerformingAction){
                cout << "---Perfroming Action...--- \n";
                switch (type) {
                    case 1:
                    {
                        rec_traj.printf_result();
                        break;
                    }
                    default:
                    {
                        rec_traj.printf_result();
                        break;
                    }      
                }          
            }else{
                cout << "---Not Performing Action, Normal Flight.--- \n";
            }

        } else {
            cout << "+++++++++++++++++++++++++++++++++++++++++++++" <<endl;
            ROS_WARN(">>>>>>> Danger, Switch to MoveENU !!<<<<<<<<<");
            cout << "+++++++++++++++++++++++++++++++++++++++++++++" <<endl;
        }
        drone.printf_state();
    }

    // print out controller parameters
    void payload_controller_JGCD::printf_param()
    {    
        cout <<">>>>>>>> Parameter For Payload Stabilization Controller (JGCD 2020) <<<<<<<<<" <<endl;
        drone.printf_param();
  
        cout <<"Payload_MASS : "<< Payload_Mass << " [kg] " << endl;
        cout <<"TotalLiftedMass" << TotalLiftedMass << " [kg] "<<endl;
        cout <<"Cable_Length : "<< Cable_Length << " [m] "  <<  endl;
        cout <<"Num of Drones: " << num_drone <<endl;
        cout <<"Tether Offset x : "<< TetherOffset(math_utils::Vector_X) << " [m] ";
        cout <<"Tether Offset y : "<< TetherOffset(math_utils::Vector_Y) << " [m] ";
        cout <<"Tether Offset z : "<< TetherOffset(math_utils::Vector_Z) << " [m] ";
        cout << endl;
        cout << "D: " <<D <<endl;
        cout << "Ej: " <<Ej <<endl;
        cout <<" a_j : " <<PayloadSharingPortion<<endl;

        cout <<" Basic Control Gains:  " <<endl;
        cout <<" kv_x : "<< kv(math_utils::Vector_X,math_utils::Vector_X) << endl;
        cout <<" kv_y : "<< kv(math_utils::Vector_Y,math_utils::Vector_Y) << endl;
        cout <<" kv_z : "<< kv(math_utils::Vector_Z,math_utils::Vector_Z) << endl;
        cout <<" kR_x : "<< kR(math_utils::Vector_X,math_utils::Vector_X) <<endl;
        cout <<" kR_y : "<< kR(math_utils::Vector_Y,math_utils::Vector_Y) <<endl;
        cout <<" kR_z : "<< kR(math_utils::Vector_Z,math_utils::Vector_Z) <<endl;  
        cout <<" kphi_x : " << Kphi(math_utils::Vector_X,math_utils::Vector_X) <<endl;
        cout <<" kphi_y : " << Kphi(math_utils::Vector_Y,math_utils::Vector_Y) <<endl;  
        cout <<" kphi_z : " << Kphi(math_utils::Vector_Z,math_utils::Vector_Z) <<endl;    
        if (isPubAuxiliaryState) {
            cout << "AuxiliaryState has been published..." << endl;
        } else {
            cout << "AuxiliaryState NOT published!!" << endl;
        }

        // Display estimation parameter:
        cout <<" UDE parameter:  " << endl;
        cout << " lambda_jx: " << lambda_j(math_utils::Vector_X,math_utils::Vector_X) 
             << " lambda_jy: " << lambda_j(math_utils::Vector_Y,math_utils::Vector_Y)
             << " lambda_jz: " << lambda_j(math_utils::Vector_Z,math_utils::Vector_Z) << endl;
        // Display control limitation:
        cout <<" Control Limit:  " <<endl;
        cout <<" pxy_error_max : "<< pos_error_max[math_utils::Vector_X] << endl;
        cout <<" pz_error_max :  "<< pos_error_max[math_utils::Vector_Z] << endl;
        // maximum lift
        cout << " fpmax_x : " << fp_max(math_utils::Vector_X) 
             << " fpmax_y : " << fp_max(math_utils::Vector_Y) 
             << " fpmax_z : " << fp_max(math_utils::Vector_Z) <<endl;

        switch (type) {
            case 1:
            {
                rec_traj.printf_param();
                break;
            }
            default:
            {
                rec_traj.printf_param();
                break;
            }
        }
        // send the parameters to ground station:
        send_parameter_to_ground_station();
    }
}
#endif
