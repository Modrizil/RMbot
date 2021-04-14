#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_I 0x69
#define KEYCODE_SPACE 0x20
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
        {
          *c = KEYCODE_B;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
        {
          *c = KEYCODE_C;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
        {
          *c = KEYCODE_E;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
        {
          *c = KEYCODE_F;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
        {
          *c = KEYCODE_G;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
        {
          *c = KEYCODE_T;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
        {
          *c = KEYCODE_V;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class RMbotTeleop
{
public:
  RMbotTeleop(ros::NodeHandle& nh, double l_scale, double a_scale);
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linearX_, linearY_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

RMbotTeleop::RMbotTeleop(ros::NodeHandle& nh, double l_scale=50, double a_scale=50):
  nh_(nh),
  linearX_(0),
  linearY_(0),
  angular_(0),
  l_scale_(l_scale),
  a_scale_(a_scale)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmbot_teleop");
  ros::NodeHandle nh;
  RMbotTeleop teleop(nh);

  signal(SIGINT,quit);

  teleop.keyLoop();
  quit(0);
  
  return(0);
}


void RMbotTeleop::keyLoop()
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use j k l i u o to move the rmbot. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
      // printf("%c -> 0x%02X\n", c,c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    linearX_=linearY_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_J:
        ROS_DEBUG("+ Y");
        // angular_ = 1.0;
        linearY_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_K:
        ROS_DEBUG("- X");
        // angular_ = -1.0;
        linearX_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_I:
        ROS_DEBUG("+ X");
        linearX_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_L:
        ROS_DEBUG("- Y");
        linearY_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("+ W");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_O:
        ROS_DEBUG("- W");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_SPACE:
        ROS_DEBUG("stop");
        linearY_ = 0;
        linearX_ = 0;
        angular_ = 0;
        dirty = true;
        break;
      
      case KEYCODE_Q:
        ROS_DEBUG("quit");
        return;
    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linearX_;
    twist.linear.y = l_scale_*linearY_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}