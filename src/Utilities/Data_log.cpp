
//头文件
#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <iomanip>

//msg 头文件
#include <px4_command/data_log.h>
#include <nav_msgs/Odometry.h>
using namespace std;

px4_command::data_log data_log;
nav_msgs::Odometry truth;
void save_flight_data(std::ofstream& out_file, float timenow);                       //储存数据函数
float get_dt(ros::Time last);
void data_log_cb(const px4_command::data_log::ConstPtr& msg)
{
    data_log = *msg;
}
void truth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    truth = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Data_log");
    ros::NodeHandle nh("~");

    //【订阅】cartographer估计位置
    ros::Subscriber data_log_sub = nh.subscribe<px4_command::data_log>("/px4_command/data_log", 1000, data_log_cb);
    ros::Subscriber truth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth_drone", 1000, truth_cb);

    // 频率
    ros::Rate rate(50.0);


    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "1 for data log， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    //储存数据

    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    char iden_path[256];
    sprintf(iden_path, "/home/fly_vision/log/Data-log-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    std::ofstream out_data_file(iden_path);

    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " pos_drone.x " << " pos_drone.y " << " pos_drone.z " \
                                << " vel_drone.x " << " vel_drone.y " << " vel_drone.z " \
                                << " pos_sp.x " << " pos_sp.y " << " pos_sp.z " \
                                << " vel_sp.x " << " vel_sp.y " << " vel_sp.z " \
                                << " u_l.x " << " u_l.y " << " u_l.z " \
                                << " u_d.x " << " u_d.y " << " u_d.z " \
                                << " u_total.x " << " u_total.y " << " u_total.z " \
                                << " thrust_sp.x " << " thrust_sp.y " << " thrust_sp.z " \
                                << " truth.x " << " truth.y " << " truth.z " \
                                << " truth.vx " << " truth.vy " << " truth.vz " \
                                <<std::endl;
    }

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        float cur_time = get_dt(begin_time);

        //回调一次 更新传感器状态
        ros::spinOnce();

        save_flight_data(out_data_file, cur_time);
        std::cout << "Saving Data!!" << std::endl;


        rate.sleep();
    }

    return 0;

}

void save_flight_data(std::ofstream& out_file, float timenow)
{
    out_file << timenow <<"  "<< data_log.pos[0] <<"  "<< data_log.pos[1] <<"  "<< data_log.pos[2] <<"  "\
                              << data_log.vel[0] <<"  "<< data_log.vel[1] <<"  "<< data_log.vel[2] <<"  "\
                              << data_log.pos_sp[0] <<"  "<< data_log.pos_sp[1] <<"  "<< data_log.pos_sp[2] <<"  "\
                              << data_log.vel_sp[0] <<"  "<< data_log.vel_sp[1] <<"  "<< data_log.vel_sp[2] <<"  "\
                              << data_log.u_l[0] <<"  "<< data_log.u_l[1] <<"  "<< data_log.u_l[2] <<"  "\
                              << data_log.u_d[0] <<"  "<< data_log.u_d[1] <<"  "<< data_log.u_d[2] <<"  "\
                              << data_log.u_total[0] <<"  "<< data_log.u_total[1] <<"  "<< data_log.u_total[2] <<"  "\
                              << data_log.thrust_sp[0] <<"  "<< data_log.thrust_sp[1] <<"  "<< data_log.thrust_sp[2] <<"  "\
                              << truth.pose.pose.position.x<<"  "<< truth.pose.pose.position.y <<"  "<< truth.pose.pose.position.z <<"  "\
                              << truth.twist.twist.linear.x<<"  "<< truth.twist.twist.linear.y <<"  "<< truth.twist.twist.linear.z <<"  "\
                              << std::endl;
}

//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
