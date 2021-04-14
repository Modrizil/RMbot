#include "com_controller.h"

com_controller::com_controller(com_controller& other)
    :nh_(other.nh_), dev_(other.dev_), baudrate_(other.baudrate_), time_out_(other.time_out_), hz_(other.hz_)
{}

com_controller::com_controller(ros::NodeHandle& nh, std::string dev="/dev/ttyUSB0", int baudrate=115200, int time_out = 1000, int hz = 50)
                            :nh_(nh), dev_(dev), baudrate_(baudrate), time_out_(time_out), hz_(hz), odom_state(false)
{
    if(init()){
        ROS_INFO_STREAM("Communication Controller - Initialized.");
    }else{
        ROS_ERROR_STREAM("Communication Controller - Initialization failed!!!");

    }
}

//初始化
bool com_controller::init()
{
    //set parameters
    nh_.setParam("dev", dev_);
    nh_.setParam("baudrate", baudrate_);
    nh_.setParam("time_out", time_out_);
    nh_.setParam("hz", hz_);

	ROS_INFO_STREAM("dev:   "<<dev_);
	ROS_INFO_STREAM("buad:   "<<baudrate_);
	ROS_INFO_STREAM("time_out:   "<<time_out_);
	ROS_INFO_STREAM("hz:   "<<hz_);

    try
    {
        ros_ser.setPort(dev_);
        ros_ser.setBaudrate(baudrate_);
        serial::Timeout t_o = serial::Timeout::simpleTimeout(time_out_);
        ros_ser.setTimeout(t_o);
        ros_ser.open();
        ros_ser.flushInput();
    }
    catch(const serial::IOException& e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR_STREAM("Unable to open port." << e.what());
        return false;
    }

    if(ros_ser.isOpen()){
        ros_ser.flushInput();
        ROS_INFO_STREAM("Serial Port Opened.");
        return true;
    }else
    {
        ROS_ERROR_STREAM("Serial Port is not open!!");
        return false;
    }    
}

//发送
void com_controller::send_speed_to_chassis(float x, float y, float w)
{
    uint8_t data_tem[50];
    unsigned int speed_0ffset=200; //速度偏移值 10ｍ/s，把速度转换成正数发送
    unsigned char i,counter=0;
    unsigned char  cmd,length;
    unsigned int check=0;
    cmd =0xF2;
    data_tem[counter++] =0xAE;
    data_tem[counter++] =0xEA;
    data_tem[counter++] =0x08;
    data_tem[counter++] =cmd;
    
    data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
    data_tem[counter++] =((x+speed_0ffset)*100);
    
    data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
    data_tem[counter++] =((y+speed_0ffset)*100);
    
    data_tem[counter++] =((w+speed_0ffset)*100)/256; // W
    data_tem[counter++] =((w+speed_0ffset)*100);

    // data_tem[counter++] =0xff;
    // data_tem[2] =counter-2 ;
    data_tem[counter++] =0xEF;
    data_tem[counter++] =0xFE;
    
    ros_ser.write(data_tem,counter);
}

bool com_controller::receive_uart_data()
{
    unsigned char received_tem[500];
    uint16_t len=0, i=0, j=0;

    unsigned char tem_last=0, tem_curr=0, rec_flag=0;//定义接受标志位
    uint16_t header_count=0, step=0; //计算数据中有多少帧

    uint16_t offset = 32768;

    len = serial_data.data.size(); //接受串口中的缓冲数据

    if(len < 1 || len > 500){
        ROS_INFO_STREAM("data is either too large or too short, len : "<< serial_data.data.size());
        return false;
    }
    ROS_INFO_STREAM("Received Serial Data [Message length: "<< serial_data.data.size() << "]");

    //找到所有帧位置
    for(i=0; i<len; i++){
        tem_last = tem_curr;
        tem_curr = serial_data.data.at(i);
        if(tem_last == 0xAE && tem_curr == 0xEA && rec_flag == 0){
            rec_flag = 1;
            received_tem[j++] = tem_last;
            received_tem[j++] = tem_curr;
            // ROS_INFO_STREAM("Found frame head...");
        }else if(rec_flag == 1){
            received_tem[j++] = serial_data.data.at(i);
            if(tem_last == 0xEF && tem_curr == 0xFE){
                header_count++;
                rec_flag = 2;
            }
        }else{
            rec_flag = 0;
        }
    }

    //接收所有数据
    step = 0;
    // for(i=0; i<header_count; i++){
    for(i=0; i<1; i++){          //每一次接收数据中，只接收1帧数据，每一次接收的数据长度大约13帧，改为只解析其中的一帧，可以有效的降低cpu的计算资源
        len = received_tem[2+step] + 4;
        // ///////////////
        // for(int tmp = 0; tmp < 30; tmp++){
        //     printf("0x%02X ", received_tem[step+tmp]);
        // }
        // printf("\n");
        // ///////////////
        if(received_tem[0+step] == 0xAE && received_tem[1+step] == 0xEA
        && received_tem[len-2+step] == 0xEF && received_tem[len-1+step] == 0xFE){
            if(received_tem[3+step] == 0x02){
                //stm32中轮胎对应关系 左上[0] 左下[2] 右下[1] 右上[3]
                C_speed.wheelSpeed[0] = received_tem[4+step]*256 + received_tem[5+step] - offset;//左上
                C_speed.wheelSpeed[1] = received_tem[8+step]*256 + received_tem[9+step] - offset;//左下
                C_speed.wheelSpeed[2] = received_tem[6+step]*256 + received_tem[7+step] - offset;//右下
                C_speed.wheelSpeed[3] = received_tem[10+step]*256 + received_tem[11+step] - offset;//右上
                wheelSpeed2chassisSpeed();
                C_speed.angle[0] = received_tem[12+step]*256 + received_tem[13+step] - offset;
                C_speed.angle[1] = received_tem[16+step]*256 + received_tem[17+step] - offset;
                C_speed.angle[2] = received_tem[14+step]*256 + received_tem[15+step] - offset;
                C_speed.angle[3] = received_tem[18+step]*256 + received_tem[19+step] - offset;
                
                short tmpangle = received_tem[20+step]*256 + received_tem[21+step];
                C_speed.yawangle = ((float) tmpangle)/100.0-180;
                C_speed.zerocircle = received_tem[22+step]*256 + received_tem[23+step] - offset;
                
                ROS_INFO_STREAM("angle&yaw: "<<
                                C_speed.angle[0] << ","<<
                                C_speed.angle[1]<< ","<<
                                C_speed.angle[2]<< ","<<
                                C_speed.angle[3]<< "\t"<<
                                C_speed.yawangle<< "," << C_speed.zerocircle);

            }else if(received_tem[3+step] == 0x01){         // TODO: 需要初始化yaw
                C_origin.angle[0] = received_tem[4+step]*256 + received_tem[5+step] - offset;
                C_origin.angle[1] = received_tem[8+step]*256 + received_tem[9+step] - offset;
                C_origin.angle[2] = received_tem[6+step]*256 + received_tem[7+step] - offset;
                C_origin.angle[3] = received_tem[10+step]*256 + received_tem[11+step] - offset;
                odom_state = true;
                std::cout<<"Initialized odometry state: "<< C_origin.angle[0]<<"\t"<< C_origin.angle[1]<<"\t"<< C_origin.angle[2]<<"\t"<< C_origin.angle[3] << std::endl;
            }
            
        }else{
            ROS_WARN_STREAM("Failed to parse the frame!!!");
            return false;
        }
        step += len;
    }
    return true;


}

void com_controller::wheelSpeed2chassisSpeed()
{
    int v1 = C_speed.wheelSpeed[0];
    int v2 = C_speed.wheelSpeed[1];
    int v3 = C_speed.wheelSpeed[2];
    int v4 = C_speed.wheelSpeed[3];

    float K4_1 = 1.0 / (4.0 * WHEEL_K);
    
    C_speed.x = 0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
    C_speed.y = -0.25*v1+ 0.25*v2- 0.25*v3+ 0.25*v4;
    C_speed.w = -K4_1*v1-K4_1*v2+K4_1*v3+ K4_1*v4;
}


void com_controller::close_port()
{
    if(ros_ser.isOpen()){
        ros_ser.close();
        ROS_INFO_STREAM("serial port closed.");
    }
}

void com_controller::wait_till_available(uint8_t buffer_size)
{
    while(ros_ser.available() <= buffer_size)
    {
        sleep(0.001);
    }
}

bool com_controller::read_serial_data()
{
    bool receive_flag = false;
    if(ros_ser.available())
    {
        serial_data.data = ros_ser.read(ros_ser.available());
        receive_flag = receive_uart_data();
    }
    return receive_flag;
}

void com_controller::clear_buffer()
{
    ros_ser.flushInput();
}

void com_controller::get_initial_odometry_chassis()
{
    uint8_t data_tem[12];
    uint8_t cmd = 0xE1;

    data_tem[0] = 0xAE;
    data_tem[1] = 0xEA;
    data_tem[2] = 0x08;
    data_tem[3] = cmd;
    data_tem[4] = 0x00;
    data_tem[5] = 0x00;
    data_tem[6] = 0x00;
    data_tem[7] = 0x00;
    data_tem[8] = 0x00;
    data_tem[9] = 0x00;
    data_tem[10] = 0xEF;
    data_tem[11] = 0xFE;
    ros_ser.write(data_tem,12);
}

void com_controller::get_initial_odom_from_cspeed()
{
    C_origin.angle[0] = C_speed.angle[0];
    C_origin.angle[1] = C_speed.angle[1];
    C_origin.angle[2] = C_speed.angle[2];
    C_origin.angle[3] = C_speed.angle[3];

    odom_state = true;
    
    ROS_INFO_STREAM("Initialize from the chassis speed...");
    std::cout<<"Initialized odometry state: "<< C_origin.angle[0]<<"\t"<< C_origin.angle[1]<<"\t"<< C_origin.angle[2]<<"\t"<< C_origin.angle[3] << std::endl;
}

bool com_controller::odometry_state_got()
{
    return odom_state;
}