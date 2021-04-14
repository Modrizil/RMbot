#include "chassis_tfpub.h"
// #include "com_controller.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "RMbot_chassis_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("NodeHandle initialized.");

    int hz = 100;
    ros::Rate rate(hz);
  

    com_controller comCtrller(nh, "/dev/ttyUSB0", 115200, 1000, hz);

    // //test send speed
    // comCtrller.send_speed_to_chassis(50, 0, 0);
    // comCtrller.send_speed_to_chassis(50, 0, 0);

    chassis_TFPub cTFpub(nh, rate, "cmd_vel", comCtrller);
    cTFpub.mainloop();
      
    
    comCtrller.close_port();

    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}