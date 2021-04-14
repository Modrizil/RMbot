
/*
 * 根据命令发送麦克纳姆轮，每个轮的转速
 * 
 */

#include <iostream>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <geometry_msgs/Twist.h>
// #include <std_msgs/String.h>


#define PI 3.1415926

#define WHEEL_RATIO 19.0 // 电机和轮子的转速比
#define WHEEL_K 0.355 // abs(X) + abd(Y)
#define WHEEL_D 0.1525 //轮子直径
#define WHEEL_R (WHEEL_D/2) //轮子半径



void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);


int main(int argc, char ** argv){
    ros::init(argc, argv, "RMbot_cmd_sub");
    ros::NodeHandle nh;

    ros::Subscriber cmd_sub = nh.subscribe("RMbot/cmd_vel", 10, &cmd_vel_callback);

    ros::spin();

    ROS_INFO_STREAM("hello RMbot");

    return 0;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    //假设小车只在平面上移动，
    //所以只有水平向x、y方向的速度
    //和绕z轴的角速度
    float x_Lvelocity, y_Lvelocity, z_Avelocity;
    x_Lvelocity = msg->linear.x;
    y_Lvelocity = msg->linear.y;
    z_Avelocity = msg->angular.z;

    //计算每个轮子的线速度 m/s
    float v1, v2, v3, v4;
    v1 =x_Lvelocity-y_Lvelocity-WHEEL_K*z_Avelocity;       
    v2 =x_Lvelocity+y_Lvelocity-WHEEL_K*z_Avelocity;
    v3 =-(x_Lvelocity-y_Lvelocity+WHEEL_K*z_Avelocity);
    v4 =-(x_Lvelocity+y_Lvelocity+WHEEL_K*z_Avelocity);

    //计算轮子的转速 每秒转多少圈
    v1 =v1/(2.0*WHEEL_R*PI);    //转换为轮子的速度　RPM
    v2 =v2/(2.0*WHEEL_R*PI);
    v3 =v3/(2.0*WHEEL_R*PI);
    v4 =v4/(2.0*WHEEL_R*PI);
    
    v1 =v1*WHEEL_RATIO*60;    //转换为电机速度　单位　ＲＰＭ
    v2 =v2*WHEEL_RATIO*60;
    v3 =v3*WHEEL_RATIO*60;
    v4 =v4*WHEEL_RATIO*60;

    ROS_INFO_STREAM("v1: "<<v1<<"\tv2: "<<v2<<"\tv3: "<<v3<<"\tv4: "<<v4);
    ROS_INFO_STREAM("speed_x:"<<msg->linear.x<<"\tspeed_y:"<<msg->linear.y<<"\tspeed_w:"<<msg->angular.z);
}