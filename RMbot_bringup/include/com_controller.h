#ifndef __COM_CONTROLLER__
#define __COM_CONTROLLER__

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#define WHEEL_D 0.15
#define WHEEL_K 0.4 //abs(x) + abd(y)
#define CLEARED true
#define UNCLEARED false

struct ChassisSpeed{
    short wheelSpeed[4];
    float x;
    float y;
    float w;

    short angle[4];
    int round_count[4];
    float yawangle;
    short zerocircle;
};

class com_controller
{
    public:
        com_controller(com_controller& other);
        com_controller(ros::NodeHandle& nh,std::string dev, int baudrate, int time_out, int hz);
        bool init();
        void send_speed_to_chassis(float x, float y, float w);
        
        void get_initial_odometry_chassis();
        void get_initial_odom_from_cspeed();
        bool odometry_state_got();

        bool receive_uart_data();
        void wheelSpeed2chassisSpeed();

        void close_port();

        void wait_till_available(uint8_t buffer_size=100);
        bool read_serial_data();
        void clear_buffer();

        ChassisSpeed C_speed;
        ChassisSpeed C_origin;

    private:
        ros::NodeHandle nh_;
        serial::Serial ros_ser;
        std::string dev_;
        int baudrate_;
        int time_out_;
        int hz_;

        bool odom_state;
        std_msgs::String serial_data;
};

#endif