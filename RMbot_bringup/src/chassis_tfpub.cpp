#include "chassis_tfpub.h"

/*
 * 初始化对象
 */

chassis_TFPub::chassis_TFPub(ros::NodeHandle& nh, ros::Rate& r, std::string cmdVelTopic, com_controller& comCtrller):
                                                                nh_(nh), rate(r), cmdVelTopic_(cmdVelTopic), //comCtrller_(comCtrller),
                                                                x(0), y(0), z(0),
                                                                roll(0), pit(0), yaw(0)
{
    comCtrllerPtr_ = &comCtrller;
    corigin = &(comCtrllerPtr_->C_origin);
    corigin->round_count[0] = corigin->round_count[1] = corigin->round_count[2] = corigin->round_count[3] = 0;
    
    cspeed = &(comCtrllerPtr_->C_speed);
    cspeed->round_count[0] = cspeed->round_count[1] = cspeed->round_count[2] = cspeed->round_count[3] = 0;

    Wlast.mileage[0] = Wlast.mileage[1] = Wlast.mileage[2] = Wlast.mileage[3] = 0;
    Wcurrent.mileage[0] = Wcurrent.mileage[1] = Wcurrent.mileage[2] = Wcurrent.mileage[3] = 0;

    pos.position_w = 0;
    pos.position_x = 0;
    pos.position_y = 0;
    
    setOdomTopic("odom");
    init_pub();
    init_sub();
    ROS_INFO_STREAM("chassis_TFPub start...");
}


/*
 * 初始化 publisher , subscriber
 */

void chassis_TFPub::init_pub()
{
    pub_ = nh_.advertise<nav_msgs::Odometry>(odomTopic_, 20);
}

void chassis_TFPub::init_sub()
{
    sub_ = nh_.subscribe(cmdVelTopic_, 1, &chassis_TFPub::VelCallback, this);
}

/*
 * 订阅速度控制命令后的回调函数
 */

void chassis_TFPub::VelCallback(const geometry_msgs::TwistPtr& vel)
{
    x = vel->linear.x;
    y = vel->linear.y;
    z = vel->linear.z;

    roll = vel->angular.x;///pi*180;
    pit = vel->angular.y;///pi*180;
    yaw = vel->angular.z;///pi*180;

    // send speed cmd to chassis
    ROS_INFO_STREAM("x: "<<x<<"\ty: "<<y<<"\tyaw: "<<yaw);
    comCtrllerPtr_->send_speed_to_chassis(x, y, yaw);
    comCtrllerPtr_->send_speed_to_chassis(x, y, yaw);
    // rate.sleep();
}

/*
 * 设置发布的话题
 * 发布里程数据和坐标转换树
 */

void chassis_TFPub::setOdomTopic(std::string topic)
{
    odomTopic_ = topic;
}

void chassis_TFPub::publish_odom(float posx, float posy, float posw, 
                                 float velx, float vely, float velw)
{
    geometry_msgs::TransformStamped trans;
    geometry_msgs::Quaternion q;
    q = tf::createQuaternionMsgFromYaw(posw);
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = posx;
    trans.transform.translation.y = posy;
    trans.transform.rotation = q;
    tfbrd_.sendTransform(trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = posx;
    odom.pose.pose.position.y = posy;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = q;
    odom.twist.twist.linear.x = velx;
    odom.twist.twist.linear.y = vely;
    odom.twist.twist.angular.z = velw;
    pub_.publish(odom);
}


//////////////////////////////////////////////////////////////////////////////////
/*
 * 对象的主要循环
 */

void chassis_TFPub::mainloop()
{
    comCtrllerPtr_->get_initial_odometry_chassis();
    comCtrllerPtr_->get_initial_odometry_chassis();
    
    comCtrllerPtr_->wait_till_available(30);
    bool receive_flag = comCtrllerPtr_->read_serial_data();
    if(!comCtrllerPtr_->odometry_state_got()){
        ROS_WARN_STREAM("Unable to get initial odometry state");
        // return;
        comCtrllerPtr_->get_initial_odom_from_cspeed();
    }
    init_clast();

    // pause and continue by enter
    char c[20];
    std::cin>> c;

    while(ros::ok())
    {
        comCtrllerPtr_->wait_till_available();
        bool receive_flag = comCtrllerPtr_->read_serial_data();
        if(receive_flag)
        {
            rndCtn_update();
            angle_update();

            // ROS_INFO_STREAM("rndctn:"<<cspeed->round_count[0]<<","<<
            //                         cspeed->round_count[1]<<","<<
            //                         cspeed->round_count[2]<<","<<
            //                         cspeed->round_count[3]);
            calculate_position();

        }else
        {
            comCtrllerPtr_->clear_buffer();
        }
        ros::spinOnce();
        rate.sleep();
    }
}



/* 计算小车的坐标
 * 角度更新，将上一个信息等于现在的数据
 * 计算轮子转的圈数
**/

void chassis_TFPub::init_clast()
{
    for(int i = 0; i< 4; i++){
        clast->angle[i] = corigin->angle[i];
        clast->round_count[i] = corigin->round_count[i];
    }
    clast->yawangle = corigin->yawangle;
    clast->zerocircle = corigin->zerocircle;

    ROS_INFO_STREAM("Init clast:"<<clast->round_count[0]<<","<<
                                clast->round_count[1]<<","<<
                                clast->round_count[2]<<","<<
                                clast->round_count[3]);
}

void chassis_TFPub::calculate_position()
{
    //计算每个轮子的转动位移，利用转换矩阵合成x,y,w三个方向的位移
    float delta[4] = {0};
    float K4_1 = 1.0/(4.0*WHEEL_K);
    float pos_x_delta, pos_y_delta, pos_w_delta;
    float linear_x, linear_y, angular_w;
    
    Wlast.mileage[0] = Wcurrent.mileage[0];
    Wlast.mileage[1] = Wcurrent.mileage[1];
    Wlast.mileage[2] = Wcurrent.mileage[2];
    Wlast.mileage[3] = Wcurrent.mileage[3];
    
    // angle [0][1] 角度正增
    Wcurrent.mileage[0] = cspeed->round_count[0]*WHEEL_D*pi/19 + (cspeed->angle[0]-corigin->angle[0])*WHEEL_D*pi/8191/19;
    Wcurrent.mileage[1] = cspeed->round_count[1]*WHEEL_D*pi/19 + (cspeed->angle[1]-corigin->angle[1])*WHEEL_D*pi/8191/19;
    // angle [2][3] 角度正减
    Wcurrent.mileage[2] = cspeed->round_count[2]*WHEEL_D*pi/19 - (cspeed->angle[2]-corigin->angle[2])*WHEEL_D*pi/8191/19;
    Wcurrent.mileage[3] = cspeed->round_count[3]*WHEEL_D*pi/19 - (cspeed->angle[3]-corigin->angle[3])*WHEEL_D*pi/8191/19;
    
    delta[0] = Wcurrent.mileage[0] - Wlast.mileage[0];
    delta[1] = Wcurrent.mileage[1] - Wlast.mileage[1];
    delta[2] = Wcurrent.mileage[2] - Wlast.mileage[2];
    delta[3] = Wcurrent.mileage[3] - Wlast.mileage[3];

    pos_x_delta = 0.25*delta[0] + 0.25*delta[1] + 0.25*delta[2] + 0.25*delta[3];
    pos_y_delta = -0.25*delta[0] + 0.25*delta[1] - 0.25*delta[2] + 0.25*delta[3];
    pos_w_delta = -K4_1*delta[0] - K4_1*delta[1] + K4_1*delta[2] + K4_1*delta[3];
    
    pos.position_x = pos.position_x + cos(pos.position_w)*pos_x_delta - sin(pos.position_w)*pos_y_delta;
    pos.position_y = pos.position_y + sin(pos.position_w)*pos_x_delta + cos(pos.position_w)*pos_y_delta;
    pos.position_w = pos.position_w + pos_w_delta;

    if(pos.position_w > 2*pi){
        pos.position_w = pos.position_w - 2*pi;
    }else if(pos.position_w < -2*pi){
        pos.position_w = pos.position_w + 2*pi;
    }

    ros::Duration t_dur = rate.cycleTime();
    double t_delta = t_dur.toSec();

    linear_x = pos_x_delta / t_delta;
    linear_y = pos_y_delta / t_delta;
    angular_w = pos_w_delta / t_delta;
    
    // ROS_INFO_STREAM("x: "<<pos.position_x<<", "<<
    //                 "y: "<<pos.position_y<<", "<<
    //                 "w: "<<pos.position_w<<"\t"<<
    //                 "vel: "<<linear_x<<", "<<linear_y<<", "<<angular_w);
    ROS_INFO("x: %.02f, y: %.02f, w: %.02f\t vel: [%.02f, %.02f, %.02f]", pos.position_x, pos.position_y, pos.position_w, linear_x, linear_y, angular_w);
    
    // 计算数据之后，进行发布
    publish_odom(pos.position_x, pos.position_y, pos.position_w, 
                 linear_x, linear_y, angular_w);
    
}

void chassis_TFPub::angle_update()
{
    for(int i = 0; i< 4; i++){
        clast->angle[i] = cspeed->angle[i];
        clast->round_count[i] = cspeed->round_count[i];
    }
    clast->yawangle = cspeed->yawangle;
    clast->zerocircle = cspeed->zerocircle;
}

void chassis_TFPub::rndCtn_update()
{
    // angle [0][1]
    // lastangle - currentangle >  7000  rndcnt++
    // lastangle - currentangle < -7000 rndcnt--
    if(clast->angle[0] - cspeed->angle[0] > 7000){
        cspeed->round_count[0] ++;
    }
    if(clast->angle[0] - cspeed->angle[0] < -7000){
        cspeed->round_count[0] --;
    }

    if(clast->angle[1] - cspeed->angle[1] > 7000){
        cspeed->round_count[1] ++;
    }
    if(clast->angle[1] - cspeed->angle[1] < -7000){
        cspeed->round_count[1] --;
    }

    // angle [2][3]
    // lastangle - currentangle >  7000  rndcnt--
    // lastangle - currentangle < -7000 rndcnt++
    if(clast->angle[2] - cspeed->angle[2] > 7000){
        cspeed->round_count[2] --;
    }
    if(clast->angle[2] - cspeed->angle[2] < -7000){
        cspeed->round_count[2] ++;
    }

    if(clast->angle[3] - cspeed->angle[3] > 7000){
        cspeed->round_count[3] --;
    }
    if(clast->angle[3] - cspeed->angle[3] < -7000){
        cspeed->round_count[3] ++;
    }

}


