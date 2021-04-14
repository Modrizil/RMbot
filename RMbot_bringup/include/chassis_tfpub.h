#ifndef __CHASSIS_TFPUB__
#define __CHASSIS_TFPUB__

#include <iostream>
#include <ros/ros.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
#include "com_controller.h"
#define pi 3.1415926

struct Position{
    float position_x;
    float position_y;
    float position_w;
};

struct Wheel_Mileage{
    float mileage[4];
};

class chassis_TFPub
{
    public:
        chassis_TFPub(ros::NodeHandle& nh, ros::Rate& r, std::string cmdVelTopic, com_controller& comCtrller);
        
        void init_pub();
        void init_sub();

        void VelCallback(const geometry_msgs::TwistPtr& vel);

        void setOdomTopic(std::string topic);
        void publish_odom(float posx, float posy, float posw, float velx, float vely, float velw);

        void mainloop();

        void init_clast();
        void calculate_position();
        void angle_update();
        void rndCtn_update();

        

    private:
        ros::NodeHandle nh_;
        std::string odomTopic_;
        std::string cmdVelTopic_;
        com_controller *comCtrllerPtr_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        tf::TransformBroadcaster tfbrd_;
        ros::Rate rate;
        double x,y,z,roll,pit,yaw; // speed command to chassis

        ChassisSpeed* corigin;
        ChassisSpeed* cspeed;
        ChassisSpeed* clast = new ChassisSpeed();
        Position pos;
        Wheel_Mileage Wlast;
        Wheel_Mileage Wcurrent;
        
        
};

#endif