/***************************************************************************************************************************
 * px4_interdrone_communication.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2019.12.02
 * Get the control force and quadrotor relative state for robust cooperative payload control 
 * if the drone is uav0 
 * subscribe the /uav#/px4_command/control_output (f_Lj)
 * subscribe  
 * publish Delta_T and Delta_R 
 * if the drone is not uav0 
 * publish f_Lj and delta_j
 * subscribe Delta_T and Delta_R from uav0 
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <math_utils.h>
#include <px4_command/DroneState.h>
#include <px4_command/AddonForce.h>
#include <px4_command/FleetStatus.h>
#include <px4_command/ControlCommand.h>
#include <px4_command/MultiPayloadAction.h>
#include <px4_command_utils.h>
#include <rectangular_trajectory.h>
#include <command_to_mavros_multidrone.h>
using std::vector;
using std::string;

// messages:
static px4_command::AddonForce  _AddonForce;// addonforce to be published
static px4_command::DroneState  _DroneState;// dronestate from uav0
static px4_command::ControlCommand Command_Now;
//estimation:
static Eigen::Vector3f Delta_T_I;
static Eigen::Vector3f Delta_R_I;
static Eigen::Vector3f Delta_T;
static Eigen::Vector3f Delta_R;
static Eigen::Vector3f Delta_rt;
static Eigen::Vector3f Delta_pt;
static Eigen::Matrix3f Delta_sq;
static bool integration_start;
static float intergration_start_height;
// states 
static Eigen::Matrix<float,24,1> X_total;
static Eigen::Matrix<float,24,1> X_total_rj_modified;
static Eigen::Matrix<float,6,1>  MPCU_total;
static Eigen::Matrix<float,6,24> MPCK;
static Eigen::Matrix<float,2,3> r_sq;
static Eigen::Matrix<float,2,3> rd_sq;
static Eigen::Matrix<float,2,3> v_sq;
static Eigen::Matrix3f f_L_sq;// 
static float cur_time;
static Eigen::Vector3f Xp;
static Eigen::Matrix3f R_IP;
static Eigen::Matrix3f R_PI;
static Eigen::Vector3f v_p;
static Eigen::Vector3f omega_p;
static Eigen::Matrix3f omega_p_cross;
// command 
static Eigen::Vector3f reference_position;
static Eigen::Vector3f vel_error;
static Eigen::Vector3f pos_error;
static Eigen::Vector3f angle_error;
static Eigen::Vector3d AttitudeTargetEuler;
static Eigen::Vector4f AttitudeTargetQuaternionv;
static Eigen::Quaterniond AttitudeTargetQuaterniond;
static Eigen::Matrix3f R_IPd ;
// auxiliary variables:
static Eigen::Vector3f FT;
static Eigen::Vector3f FR;
static Eigen::Vector3f FR2;
static Eigen::Vector3f BT;
static Eigen::Vector3f DT;
static Eigen::Vector3f DR;
static Eigen::Matrix<float,3,2> B_j;
// parameters:
static int   num_of_drones;
static float payload_mass;
static float M_q;
static Eigen::Vector3f g_I;
static Eigen::Matrix3f t_sq;
static Eigen::Matrix3f A;
static Eigen::Matrix3f J_q;
static Eigen::Matrix3f J_p;
static Eigen::Matrix3f D;
static Eigen::Vector3f quadrotor_mass;
static Eigen::Vector3f a_j_sq;
static Eigen::Matrix3f lambda_T;
static Eigen::Matrix3f lambda_R;
static Eigen::Vector3f cablelength;
static Eigen::Vector3f cablelength_squared;
static Eigen::Vector3f R1,R2;
static Eigen::Matrix<float,3, Eigen::Dynamic> E_j;
static Eigen::Matrix3f Identity;
static Eigen::Matrix3f kv,KR;
int CooperativePayload;
// action state
static bool isperformAction;
static int type;
static trajectory::Reference_Path rect_path;
static trajectory::Rectangular_Trajectory_Parameter rect_param;
static trajectory::Rectangular_Trajectory rec_traj;
// geo_fence
static Eigen::Vector2f geo_fence_x;
static Eigen::Vector2f geo_fence_y;
static Eigen::Vector2f geo_fence_z;

void GetCommand(const px4_command::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
    reference_position(math_utils::Vector_X) = Command_Now.Reference_State.position_ref[math_utils::Vector_X];
    reference_position(math_utils::Vector_Y) = Command_Now.Reference_State.position_ref[math_utils::Vector_Y];
    reference_position(math_utils::Vector_Z) = Command_Now.Reference_State.position_ref[math_utils::Vector_Z];
    AttitudeTargetEuler(0) = (double)Command_Now.Reference_State.roll_ref;
    AttitudeTargetEuler(1) = (double)Command_Now.Reference_State.pitch_ref;
    AttitudeTargetEuler(2) = (double)Command_Now.Reference_State.yaw_ref;
    AttitudeTargetQuaterniond = quaternion_from_rpy(AttitudeTargetEuler);
    AttitudeTargetQuaternionv(0) = (float)AttitudeTargetQuaterniond.w();
    AttitudeTargetQuaternionv(1) = (float)AttitudeTargetQuaterniond.x();
    AttitudeTargetQuaternionv(2) = (float)AttitudeTargetQuaterniond.y();
    AttitudeTargetQuaternionv(3) = (float)AttitudeTargetQuaterniond.z();
    R_IPd =  QuaterionToRotationMatrix(AttitudeTargetQuaternionv);
}

void GetDroneState(const px4_command::DroneState::ConstPtr& msg) {
    _DroneState = *msg;
    _DroneState.time_from_start = cur_time;
    for (int i = 0; i < 3; i ++) {
        Xp(i) = _DroneState.payload_pos[i];
    }
}

void GetUAV0Status(const px4_command::FleetStatus::ConstPtr& msg){
    px4_command::FleetStatus uav0_status;
    uav0_status = *msg;
    r_sq(0,0) = uav0_status.r_jx;
    r_sq(1,0) = uav0_status.r_jy;
    v_sq(0,0) = uav0_status.v_jx;
    v_sq(1,0) = uav0_status.v_jy;
    f_L_sq(0,0) = uav0_status.f_Ljx;
    f_L_sq(1,0) = uav0_status.f_Ljy;
    f_L_sq(2,0) = uav0_status.f_Ljz;
    Delta_sq(0,0) = uav0_status.delta_jx;
    Delta_sq(1,0) = uav0_status.delta_jy;
    Delta_sq(2,0) = uav0_status.delta_jz;
    rd_sq(0,0) = uav0_status.rd_jx;
    rd_sq(1,0) = uav0_status.rd_jy;
}
void GetUAV1Status(const px4_command::FleetStatus::ConstPtr& msg){
    px4_command::FleetStatus uav1_status;
    uav1_status = *msg;
    r_sq(0,1) = uav1_status.r_jx;
    r_sq(1,1) = uav1_status.r_jy;
    v_sq(0,1) = uav1_status.v_jx;
    v_sq(1,1) = uav1_status.v_jy;
    f_L_sq(0,1) = uav1_status.f_Ljx;
    f_L_sq(1,1) = uav1_status.f_Ljy;
    f_L_sq(2,1) = uav1_status.f_Ljz;
    Delta_sq(0,1) = uav1_status.delta_jx;
    Delta_sq(1,1) = uav1_status.delta_jy;
    Delta_sq(2,1) = uav1_status.delta_jz;
    rd_sq(0,1) = uav1_status.rd_jx;
    rd_sq(1,1) = uav1_status.rd_jy;
}
void GetUAV2Status(const px4_command::FleetStatus::ConstPtr& msg){
    px4_command::FleetStatus uav2_status;
    uav2_status = *msg;
    r_sq(0,2) = uav2_status.r_jx;
    r_sq(1,2) = uav2_status.r_jy;
    v_sq(0,2) = uav2_status.v_jx;
    v_sq(1,2) = uav2_status.v_jy;
    f_L_sq(0,2) = uav2_status.f_Ljx;
    f_L_sq(1,2) = uav2_status.f_Ljy;
    f_L_sq(2,2) = uav2_status.f_Ljz;
    Delta_sq(0,2) = uav2_status.delta_jx;
    Delta_sq(1,2) = uav2_status.delta_jy;
    Delta_sq(2,2) = uav2_status.delta_jz;
    rd_sq(0,2) = uav2_status.rd_jx;
    rd_sq(1,2) = uav2_status.rd_jy;
}

bool PayloadGeoFenceCheck(){
    if (Xp(math_utils::Vector_X) < geo_fence_x[0] || Xp(math_utils::Vector_X) > geo_fence_x[1] ||
        Xp(math_utils::Vector_Y) < geo_fence_y[0] || Xp(math_utils::Vector_Y) > geo_fence_y[1] ||
        Xp(math_utils::Vector_Z) < geo_fence_z[0] || Xp(math_utils::Vector_Z) > geo_fence_z[1]) {
        return false;
    } else {
        return true;
    }
}

bool ResponseToActionCall(px4_command::MultiPayloadAction::Request& req, px4_command::MultiPayloadAction::Response& res){

    if(PayloadGeoFenceCheck()&&req.perform_action){
        isperformAction = req.perform_action;
    }else{
        isperformAction = false;
    }
    res.status_ok = isperformAction;
    res.trajectory_type = type;
    return true;
}

void LoadMPCGain(ros::NodeHandle& nh){
    MPCK.setZero();
    float k_gain;
    string name_head = "MPCgain/Kmpc_";
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 24; j++){
            nh.param<float> (name_head + std::to_string(i) + "_" + std::to_string(j), MPCK(i,j),0.0);
        }
    }
}

void PrintEstimation(){
    cout <<">>>>>>>>  TCST 2019 Disturbance Estimator <<<<<<< \n";
    cout.setf(ios::fixed);//固定的浮点显示
    cout.setf(ios::left);//左对齐
    cout.setf(ios::showpoint);// 强制显示小数点
    cout.setf(ios::showpos);// 强制显示符号
    cout<<setprecision(3);// number of decimals 
    cout << "Current command mode is: " << _DroneState.mode << "\n";
    if(_DroneState.mode == "OFFBOARD") {
        if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization) {
            // display fleet info first
            for (int i = 0; i < num_of_drones ; i ++) {
                cout << ">>>>>>> UAV "<< i << " info <<<<<<<" << "\n";
                cout << "Delta_"<< i <<" [X Y Z] : " << Delta_sq(0,i) << " [N] "<< Delta_sq(1,i) <<" [N] "<< Delta_sq(2,i)<<" [N] "<<"\n";
                cout << "F_L_" << i << " [X Y Z] : " << f_L_sq(0,i) <<" [N] "<<f_L_sq(1,i) <<" [N] "<< f_L_sq(2,i)<<" [N] "<<"\n";
                cout << "r_" << i << " [X Y] : "<< r_sq(0,i) << " [m] " << r_sq(1,i) << " [m] "    << "\n";
                cout << "v_" << i << " [X Y] : "<< v_sq(0,i) << " [m/s] " << v_sq(1,i) << " [m/s] "<< "\n";
                cout << "rd_" << i << " [X Y] : " << rd_sq(0,i) << " [m] " << rd_sq(1,i) << " [m] " << "\n";
            }
            // then display estimation results
            cout << ">>>>>>> IN PAYLOAD STABILIZATION MODE, CALCULATING ESTIMATION <<<<<<<" << "\n";
            cout << "Delta_T [X Y Z] : " << Delta_T(0) << " [N] "<< Delta_T(1)<<" [N] "<<Delta_T(2)<<" [N] "<<"\n";
            cout << "Delta_pt [X Y Z] : " << Delta_pt(0) << " [N] "<< Delta_pt(1)<<" [N] "<<Delta_pt(2)<<" [N] "<<"\n";
            cout << "Delta_R [X Y Z] : " << Delta_R(0) << " [N] "<< Delta_R(1)<<" [N] "<<Delta_R(2)<<" [N] "<<"\n";
            cout << "Delta_rt [X Y Z] : " << Delta_rt(0) << " [N] "<< Delta_rt(1)<<" [N] "<<Delta_rt(2)<<" [N] "<<"\n";
            cout << "R1 [X Y Z] : " << R1(0) << " [N] " << R1(1) << " [N] " << R1(2) << " [N] " <<"\n";
            cout << "R2 [X Y Z] : " << R2(0) << " [N] " << R2(1) << " [N] " << R2(2) << " [N] " <<"\n";
            // display integration flag
            if( integration_start ){
                cout<<"Integration Start... \n";
            }else{
                cout<<"Integration Hold... \n";
            }

            if(isperformAction){
                cout << "--- Perfroming Action...--- \n";
                switch (type) {
                    case 1:{
                        rec_traj.printf_result();
                        break;
                        }
                    default:{
                        rec_traj.printf_result();
                        break;
                        }
                }
            }else{
                cout << "---Not Performing Action, Normal Flight.--- \n";
            }
/* 
            if(CooperativePayload==1){
                cout << "Xtotal is:\n";
                cout << X_total.transpose() << "\n";
                cout << "U mpc is:\n";
                cout << MPCU_total.transpose() <<"\n";
                cout <<" vel error: " <<"\n";
                cout <<vel_error <<"\n";
                cout <<" pos error: " <<"\n";
                cout <<pos_error <<"\n";   
                cout <<" omega_p error: " <<"\n";
                cout <<omega_p <<"\n";
                cout <<" angle_error error: " <<"\n";
                cout <<angle_error <<"\n";                              
                for(int i = 0; i < num_of_drones; i ++) {
                    cout <<"v_"<<i<<":\n";
                    cout<< v_sq.col(i)<<"\n";// v_j
                    cout <<"r_"<<i<<":\n";
                    cout<< r_sq.col(i)- rd_sq.col(i)<<"\n";// r_j
                }

                
            }*/

        } else {
            cout << ">>>>>>> NOT IN PAYLOAD STABILIZATION MODE, ESTIMATION PAUSED <<<<<<< \n";
        }
    } else {
        cout << ">>>>>>> NOT IN OFFBOARD MODE <<<<<<< \n";
    }
}

void DisplayParameters() {
    cout <<">>>>>>>> Parameter For UDE Estimator  <<<<<<<<<" <<endl;
    cout <<"Estimator Time Constants: "<< endl;
    cout <<"lambda_T: "<< lambda_T(0,0) << " " << lambda_T(1,1) << " " << lambda_T(2,2) << " " << endl;
    cout <<"lambda_R: "<< lambda_R(0,0) << " " << lambda_R(1,1) << " " << lambda_R(2,2) << " " << endl;
    cout <<"Payload Mass: " << payload_mass << " [kg]" <<endl;
    cout << num_of_drones << " drones are used ! "<< endl;
    for (int i = 0; i< num_of_drones ; i ++) {
        cout << ">>>>>>> ---- <<<<<<<<" <<endl;
        cout <<"Quadrotor " << i << " mass: " << quadrotor_mass(i) << " [kg] "<<  endl;
        cout << "a_" << i << " = " << a_j_sq(i) << endl;
        cout << "Tether Point [X Y Z]: "<< t_sq(0,i) << " [m] " << t_sq(1,i) << " [m] " << t_sq(2,i) << " [m] " << endl;
        cout << "cablelength_" << i << " = " << cablelength(i) << " [m] " <<endl;
        cout << "E_j matrix is : " << endl;
        cout << E_j.block(0,i*3,3,3) <<endl;
    }
    cout << "Total Quadrotor Mass: " << M_q << " [kg] " << endl;
    cout << "J_q : " << endl;
    cout <<  J_q << endl;
    cout << "J_p : " << endl;
    cout <<  J_p << endl;
    cout << "A : " << endl;
    cout << A << endl;
    if(CooperativePayload!=0){
        switch (CooperativePayload){
            case 1:{
                cout <<" Using MPC control law...." <<endl;
                cout <<" MPC Gain (from col 0 - col 11 )is: " <<endl;
                cout << MPCK.block<6,12>(0,0) << endl;
                cout <<" MPC Gain (from col 12 - col 23 )is: " <<endl;
                cout << MPCK.block<6,12>(0,12) << endl;    
                break;
            }
            default:{
                cout <<" Using MPC control law...." <<endl;
                cout <<" MPC Gain (from col 0 - col 11 )is: " <<endl;
                cout << MPCK.block<6,12>(0,0) << endl;
                cout <<" MPC Gain (from col 12 - col 23 )is: " <<endl;
                cout << MPCK.block<6,12>(0,12) << endl;             
                break;
            }
        }
    }

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
    cout<< "Payload geofence is :" << endl;
    cout<< "Geofence_x_min: " << geo_fence_x(0) << " m "<<endl;
    cout<< "Geofence_x_max: " << geo_fence_x(1) << " m "<<endl;
    cout<< "Geofence_y_min: " << geo_fence_y(0) << " m "<<endl;
    cout<< "Geofence_y_max: " << geo_fence_y(1) << " m "<<endl;
    cout<< "Geofence_z_min: " << geo_fence_z(0) << " m "<<endl;
    cout<< "Geofence_z_max: " << geo_fence_z(1) << " m "<<endl;
    cout<< "intergration start height is: " << intergration_start_height << " m " <<endl;
}

void MPCDummy( Eigen::Vector3f& R1, Eigen::Vector3f& R2) {


    // calculate angluar error
    angle_error = 0.5* Veemap(R_IPd.transpose()*R_IP- R_IP.transpose() * R_IPd);

    // first determine the geo_fence is satisfied
    if(!PayloadGeoFenceCheck()){// if out of bound, return to normal mode
        isperformAction = false;
    }
    // calculate quadrotor position and velocity error:
    if(isperformAction){
        switch (type) {
            case 1:{
                rect_path = rec_traj.UpdatePosition(Xp);
                vel_error = v_p - rect_path.vd*rect_path.n;
                pos_error = (Identity - rect_path.n*rect_path.n.transpose())*(Xp - rect_path.P);
                break;
            }
        default:{
                rect_path = rec_traj.UpdatePosition(Xp);
                vel_error = v_p - rect_path.vd*rect_path.n;
                pos_error = (Identity - rect_path.n*rect_path.n.transpose())*(Xp- rect_path.P);
            break;
        }
      }
    }else{
        pos_error = Xp - reference_position;
        vel_error = v_p; // position stabilization
    }
    R1.setZero();
    R2.setZero();
    // step 1 convert all r_j and v_j into body-fixed frame    
    // assemble xe vector
    X_total.segment<3>(0) = vel_error; // vp
    X_total.segment<3>(3) = omega_p; // omega_p
    X_total.segment<3>(12) = pos_error ; // ex
    X_total.segment<3>(15) = angle_error ; // ex
    for(int i = 0; i < num_of_drones; i ++) { // r_j v_j
           X_total.segment<2>(6+2*i)= v_sq.col(i);// v_j
           X_total.segment<2>(18+2*i) = r_sq.col(i) - rd_sq.col(i); //r_j
    }
    R1 = MPCK.topRows<3>() *  X_total;
    // for R2, rotation matric should be included
    X_total_rj_modified = X_total;
    for(int i = 0; i < num_of_drones; i++) {
          X_total_rj_modified.segment<2>(6+2*i)= R_PI.topLeftCorner<2,2>()*v_sq.col(i);// 
          X_total_rj_modified.segment<2>(18+2*i) = R_PI.topLeftCorner<2,2>()*(r_sq.col(i) - rd_sq.col(i)); 
    }
    R2 = MPCK.bottomRows<3>() * X_total_rj_modified;
}

void ResetStates() {
    Delta_T_I.setZero();
    Delta_R_I.setZero();
    Delta_T.setZero();
    Delta_R.setZero();
    Delta_pt.setZero();
    Delta_rt.setZero();
    R1.setZero();
    R2.setZero();
}

int main(int argc, 
         char **argv) 
{
    ros::init(argc, argv, "px4_interdrone_communication");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    // initialize all parameters:
    integration_start = false;
    R_IPd.setZero();
    reference_position.setZero();
    vel_error.setZero();
    pos_error.setZero();
    angle_error.setZero();
    Identity.setIdentity();
    lambda_T.setZero();
    lambda_R.setZero();
    A.setZero();
    J_q.setZero();
    a_j_sq.setZero();
    g_I << 0.0,
           0.0,
          -9.81;// ENU coordinates are used !!
    /* an estimation of payload moment of inertia. accurate moment of inertia is unecessary because more payload will be added during flight test*/
    J_p << 0.1, 0.0, 0.0,
           0.0, 0.1, 0.0,
           0.0, 0.0, 0.1;
    float kL = 0;

    // loading all parameters into the estimator:
    nh.param<float> ("IntegrationStartHeight", intergration_start_height,0.0);
    nh.param<int>   ("Pos_GNC/num_drone",num_of_drones,1);
    nh.param<float> ("Payload/mass", payload_mass, 1.0);

    nh.param<float> ("Pos_GNC/lambda_Txy", lambda_T(0,0),0.2);
    nh.param<float> ("Pos_GNC/lambda_Txy", lambda_T(1,1),0.2);
    nh.param<float> ("Pos_GNC/lambda_Tz", lambda_T(2,2),0.2);
    nh.param<float> ("Pos_GNC/lambda_Rxy", lambda_R(0,0),0.2);
    nh.param<float> ("Pos_GNC/lambda_Rxy", lambda_R(1,1),0.2);
    nh.param<float> ("Pos_GNC/lambda_Rz", lambda_R(2,2),0.2);
    nh.param<float> ("Pos_GNC/kL", kL, 0.1);
    // load trajectory data
    isperformAction = false;
    type = 1;
    nh.param<float>("Rectangular_Trajectory/a_x",   rect_param.a_x, 0.0);
    nh.param<float>("Rectangular_Trajectory/a_y",   rect_param.a_y, 0.0);
    nh.param<float>("Rectangular_Trajectory/vel_x", rect_param.v_x, 0.0);
    nh.param<float>("Rectangular_Trajectory/vel_y", rect_param.v_y, 0.0);
    nh.param<float>("Rectangular_Trajectory/h",     rect_param.h, 0.0);
    nh.param<float>("Rectangular_Trajectory/center_x",     rect_param.center_x, 0.0);
    nh.param<float>("Rectangular_Trajectory/center_y",     rect_param.center_y, 0.0);
    nh.param<float>("Rectangular_Trajectory/center_z",     rect_param.center_z, 0.0);

    nh.param<float>("payload_geofence/x_min", geo_fence_x[math_utils::Vector_X], -0.6);
    nh.param<float>("payload_geofence/x_max", geo_fence_x[math_utils::Vector_Y], 0.6);
    nh.param<float>("payload_geofence/y_min", geo_fence_y[math_utils::Vector_X], -0.3);
    nh.param<float>("payload_geofence/y_max", geo_fence_y[math_utils::Vector_Y], 0.3);
    nh.param<float>("payload_geofence/z_min", geo_fence_z[math_utils::Vector_X],-0.05);
    nh.param<float>("payload_geofence/z_max", geo_fence_z[math_utils::Vector_Y], 0.6);

    rec_traj.LoadParameter(rect_param);
    // load the parameter for controller 
    nh.param<int>("CooperativePayload", CooperativePayload, 0);
    if(CooperativePayload==1){
        LoadMPCGain(nh);
        X_total.setZero();
        MPCU_total.setZero();
    }else{
        R1.setZero();
        R2.setZero();
    }
    // temp variables
    Eigen::Vector3f temp_t_j;
    M_q = 0.0;// total mass of all quadrotorsa_j_sq
    float e_n = 1.0;
    E_j.resize(3,num_of_drones*3);
    for (int i = 0; i < num_of_drones ; i ++) {
        temp_t_j.setZero();
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_x", temp_t_j(0), 0.5);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_y", temp_t_j(1), 0);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_z", temp_t_j(2), 0); 
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/PayloadSharingPortion", a_j_sq(i), 0);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/mass", quadrotor_mass(i), 1.0);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/cablelength", cablelength(i), 1.0);
        cablelength_squared(i) = cablelength(i) * cablelength(i);
        D += a_j_sq(i) * Hatmap(temp_t_j)* Hatmap(temp_t_j);
        t_sq.col(i)  = temp_t_j;
        M_q += quadrotor_mass(i);
        A += quadrotor_mass(i) * Hatmap(temp_t_j);
        J_q += - quadrotor_mass(i) * Hatmap(temp_t_j) * Hatmap(temp_t_j);
        e_n *= temp_t_j.norm();
    }
    // for 2 drone case, E_j is special

    for (int i = 0;  i< num_of_drones; i++ ) {
        if (num_of_drones < 3) { 
            E_j.block(0,i*3,3,3) = - Hatmap(t_sq.col(i)) / e_n;
        } else {
            E_j.block(0,i*3,3,3) = Hatmap(t_sq.col(i)) * D.inverse();
        }
    }

    // form the communication channels
    ros::Subscriber subUAV0status = nh.subscribe<px4_command::FleetStatus>("/uav0/px4_command/fleetstatus", 100, GetUAV0Status);
    ros::Subscriber subUAV1status = nh.subscribe<px4_command::FleetStatus>("/uav1/px4_command/fleetstatus", 100, GetUAV1Status);
    ros::Subscriber subUAV2status = nh.subscribe<px4_command::FleetStatus>("/uav2/px4_command/fleetstatus", 100, GetUAV2Status);
    ros::Publisher  pubAddonForce = nh.advertise<px4_command::AddonForce> ("/uav0/px4_command/addonforce", 1000);
    ros::Subscriber subdronestate = nh.subscribe<px4_command::DroneState>("/uav0/px4_command/drone_state", 100, GetDroneState);
    ros::Subscriber subCommand    = nh.subscribe<px4_command::ControlCommand>("/uav0/px4_command/control_command", 100, GetCommand);
    ros::ServiceServer serverAction = nh.advertiseService("/uav0/px4_command/multi_action", &ResponseToActionCall);
    // initialize all estimations
    ResetStates();
    B_j << 1.0,0.0,
           0.0,1.0,
           0.0,0.0;
    Eigen::Matrix3f t_j_cross;
    Eigen::Vector2f r_j;
    Eigen::Vector2f mu_j;
    Eigen::Vector2f v_j;
    Eigen::Vector4f AttitudeQuaternionv;
    Eigen::Vector3f temp;
    t_j_cross.setZero();
    DisplayParameters();// display parameters for checking...
    int check_flag;
    // check the data output on 
    ROS_INFO("Please check the parameter and setting, enter 1 to continue, else for quit: ");
    cin >> check_flag;
    if(check_flag != 1)
    {
        ROS_WARN("Found something wrong? terminating node ...");
        return -1;
    }

    ROS_INFO("Parameter OK, Start the interdrone communication and disturbance estimation...");
    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0.0;
    while(ros::ok())
    {
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;
        ros::spinOnce();
        switch (Command_Now.Mode) {
             case command_to_mavros_multidrone::Idle:
                // in idle mode: reset all state to zeros. 
                ResetStates();
                break;
             case command_to_mavros_multidrone::Payload_Stabilization: 
                if(_DroneState.mode != "OFFBOARD") {
                    ResetStates();// reset estimations if not in offboard mode
                } else {
                    /*Step 1, get payload information: */
                    // get the rotation matrix of the payload
                    for(int i = 0; i < 4 ;i++) {
                        AttitudeQuaternionv(i) = _DroneState.payload_quaternion[i];
                    }
                    R_IP = QuaterionToRotationMatrix(AttitudeQuaternionv); //
                    R_PI = R_IP.transpose(); // 
                    // get the linear and angular velocity of the payload
                    for (int i = 0; i < 3; i ++) {
                        v_p(i) = _DroneState.payload_vel[i];
                        omega_p(i) = _DroneState.payload_angular_vel[i];
                        omega_p_cross = Hatmap(omega_p);
                    }
                    // determine if the integration starts
                    if (Xp(math_utils::Vector_Z)>intergration_start_height){
                         integration_start = true;
                    }
                    /* Step 2 calculate force estimation */ 
                    // reset the accumulation of forces 
                    FT.setZero(); // total lift generated by props
                    FR.setZero(); // total moment generated by props
                    FR2.setZero();// auxiliary force in delta_R estimation
                    DT.setZero(); // total disturbance on quadrotors
                    DR.setZero(); // total disturbance moment on quadrotors
                    R1.setZero();
                    R2.setZero();
                    for (int i = 0; i < num_of_drones ; i ++) {
                        r_j = r_sq.col(i);
                        v_j = v_sq.col(i);
                        mu_j = kL * (r_j - rd_sq.col(i));
                        t_j_cross = Hatmap(t_sq.col(i));
                        DT += Delta_sq.col(i);
                        DR += t_j_cross * R_PI *  Delta_sq.col(i);
                        FT += f_L_sq.col(i);
                        // calculate B_j for each drone
                        float sq_r = r_j(0)*r_j(0) + r_j(1)*r_j(1);
                        if (cablelength_squared(i) - sq_r>0.01) {
                            B_j(2,0) =  - r_j(0)/sqrt((cablelength_squared(i) - sq_r));
                            B_j(2,1) =  - r_j(1)/sqrt((cablelength_squared(i) - sq_r));
                        } else {
                            B_j(2,0) = -0.1;
                            B_j(2,1) = -0.1;
                        }
                        // determine the controller type
                        if(CooperativePayload == 0){ // 0 for TCST, as cross feeding term
                            temp = B_j * (v_j + mu_j);
                            R1 += a_j_sq(i) * temp;
                            R2 += a_j_sq(i) * E_j.block(0,i*3,3,3).transpose() * R_PI * temp;                            
                        }
                        BT += quadrotor_mass(i) * B_j * v_j;
                        FR += t_j_cross * (quadrotor_mass(i)*(omega_p_cross * R_PI *B_j*v_j - omega_p_cross * t_j_cross *omega_p -  R_PI * g_I) - R_PI * f_L_sq.col(i));
                        FR2 += quadrotor_mass(i)* t_j_cross * R_PI * B_j * v_j;
                    }
                    // 1 for JGCD
                    if(CooperativePayload == 1){
                       MPCDummy(R1,R2);
                    }
                    // calculate estimation based on the TCST paper. 
                    if( integration_start ){
                        Delta_T_I +=  dt* ( Delta_T + FT + (M_q + payload_mass) * g_I + DT);
                        Delta_R_I += dt *(A* omega_p_cross * R_PI * v_p + omega_p_cross * J_p * omega_p - Delta_R +  FR - DR);
                        Delta_T = lambda_T * ( - Delta_T_I + (M_q + payload_mass) * v_p + R_IP * A.transpose() * omega_p +  BT);
                        Delta_R = lambda_R * (Delta_R_I  + A * R_PI * v_p + (J_p + J_q ) * omega_p +  FR2 );                        
                    }
                    // calculate effective disturbance: 
                    Delta_pt = constrain_vector(Delta_T, 10.0);
                    Delta_rt = constrain_vector(Delta_R, 5.0);
                }
                break;
            default:
                // in default mode, reset the estimation
                ResetStates();
                break;
         }
        PrintEstimation();
        // send the total disturbance estimation to each quadrotor
        _AddonForce.delta_Tx = Delta_pt(0);
        _AddonForce.delta_Ty = Delta_pt(1);
        _AddonForce.delta_Tz = Delta_pt(2);  
        _AddonForce.delta_Rx = Delta_rt(0);
        _AddonForce.delta_Ry = Delta_rt(1);  
        _AddonForce.delta_Rz = Delta_rt(2);
        // send cross-feeding forces to each quadrotor
        _AddonForce.R_1x = R1(0);
        _AddonForce.R_1y = R1(1);
        _AddonForce.R_1z = R1(2);
        _AddonForce.R_2x = R2(0);
        _AddonForce.R_2y = R2(1);
        _AddonForce.R_2z = R2(2);
        _AddonForce.header.stamp = ros::Time::now();
        _AddonForce.perform_action = isperformAction;
        pubAddonForce.publish(_AddonForce);
        rate.sleep();
    }
    return 0;
}

