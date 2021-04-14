#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

void publish_cmd(ros::Publisher &pub, int &flag, float x, float y, float w);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "self_circle_test");
    ros::NodeHandle nh;

    tf::TransformListener tflis;

    // geometry_msgs::PointStamped podom;
    // geometry_msgs::PointStamped pbase;
    ros::Rate r(25);

    int flag = 0;

    ros::Publisher pub_;
    pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);


    while(nh.ok()){
        tf::StampedTransform trans;
        try{
            tflis.lookupTransform("odom", "base_link", ros::Time(0), trans);
        }catch(const std::exception& e){
            ROS_ERROR("%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        float x = trans.getOrigin().x();
        float y = trans.getOrigin().y();
        // float w = trans.getRotation().w();
        float w = tf::getYaw(trans.getRotation());

        // ROS_INFO("x:%f, y:%f, w:%f", x, y, w);

        publish_cmd(pub_, flag, x, y, w);
        if(flag == -1){
            break;
        }

        r.sleep();
    }
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;
    pub_.publish(twist);
}

void publish_cmd(ros::Publisher &pub, int &flag, float x, float y, float w)
{
    geometry_msgs::Twist twist;
    float dis = 1;

    if(flag == 0){
        if(x < dis){
            twist.linear.x = 50;
            twist.linear.y = 0;
            twist.angular.z = 0;
            // ROS_INFO("cmd: [%.01f, %.01f, %.01f]", twist.linear.x, twist.linear.y, twist.angular.z);
        }else{
            flag = 1;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            ROS_WARN_STREAM("change flag to 1,\t" << x << ", "<< y << ", "<< w);
        }
    }
    else if(flag == 1){
        if(y < dis){
            twist.linear.x = 0;
            twist.linear.y = 50;
            twist.angular.z = 0;
            // ROS_INFO("cmd: [%.01f, %.01f, %.01f]", twist.linear.x, twist.linear.y, twist.angular.z);
        }else{
            flag = 2;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            ROS_WARN_STREAM("change flag to 2,\t" << x << ", "<< y << ", "<< w);
        }
    }
    else if(flag == 2){
        if(x > 0){
            twist.linear.x = -50;
            twist.linear.y = 0;
            twist.angular.z = 0;
            // ROS_INFO("cmd: [%.01f, %.01f, %.01f]", twist.linear.x, twist.linear.y, twist.angular.z);
        }else{
            flag = 3;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            ROS_WARN_STREAM("change flag to 3,\t" << x << ", "<< y << ", "<< w);
        }
    }
    else if(flag == 3){
        if(y > 0){
            twist.linear.x = 0;
            twist.linear.y = -50;
            twist.angular.z = 0;
            // ROS_INFO("cmd: [%.01f, %.01f, %.01f]", twist.linear.x, twist.linear.y, twist.angular.z);
        }else{
            flag = 4;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            ROS_WARN_STREAM("change flag to 4,\t" << x << ", "<< y << ", "<< w);
            pub.publish(twist);
        }
    }
    else if(flag == 4){
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        flag = -1;
    }

    pub.publish(twist);
    
}


