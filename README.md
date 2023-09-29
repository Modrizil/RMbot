## 1. 项目目录

```
.
├── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
├── RMbot
│   ├── map
│   │   ├── gmapping.pgm
│   │   └── gmapping.yaml
│   ├── RMbot_bringup
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   ├── chassis_tfpub.h
│   │   │   └── com_controller.h
│   │   ├── launch
│   │   │   ├── chassis.launch
│   │   │   └── teleop.launch
│   │   ├── package.xml
│   │   └── src
│   │       ├── atlas_cmd.cpp
│   │       ├── chassis_tfpub.cpp
│   │       ├── com_controller.cpp
│   │       ├── ekf_odom_pub.cpp
│   │       ├── RMbot_chassis.cpp
│   │       ├── rmbot_teleop.cpp
│   │       └── self_circle_test.cpp
│   ├── RMbot_description
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── display.launch
│   │   ├── package.xml
│   │   ├── rviz
│   │   │   └── model_view.rviz
│   │   └── urdf
│   │       ├── macros.xacro
│   │       ├── RMcar.xacro
│   │       ├── smartcar.gv
│   │       ├── smartcar.pdf
│   │       └── test.urdf
│   ├── RMbot_nav
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── amcl.launch
│   │   │   └── move_base.launch
│   │   ├── package.xml
│   │   ├── param
│   │   │   ├── base_local_planner_params.yaml
│   │   │   ├── costmap_common_params.yaml
│   │   │   ├── dwa_local_planner_params.yaml
│   │   │   ├── global_costmap_params.yaml
│   │   │   ├── local_costmap_params.yaml
│   │   │   └── move_base_params.yaml
│   │   ├── rosgraph.png
│   │   └── rviz
│   │       └── amclMovebase.rviz
│   ├── RMbot_slam
│   │   ├── CMakeLists.txt
│   │   ├── frames.gv
│   │   ├── include
│   │   │   └── RMbot_slam
│   │   ├── launch
│   │   │   └── gmapping.launch
│   │   ├── package.xml
│   │   ├── rviz_screenshot_gmapping.png
│   │   ├── src
│   │   └── tf_tree.png
```

上述路径中包含了机器人启动包(bringup)、可视化仿真包(description)、SLAM包(SLAM)、导航包(navigation)。该项目的正常运行还需要安装serial包、slam_gmapping包、cartographer_ros包，这些包的下载按照官方指南即可下载。

## 2. 常用命令

启动机器人的步骤：

1. 启动硬件设备，打开遥控器。遥控器的选项配置为3和2（即左上拨杆位于最下、右上拨杆位于中间）。当车上的遥控信号接收器为稳定绿色即可。此后关闭遥控器，机器人也能正常控制运行。
2. 打开终端，输入指令

    ```bash
    roslaunch RMbot_bringup chassis.launch
    ```

3. 如需要电脑控制小车运动，再打开一个终端，输入命令

    ```bash
    roslaunch RMbot_bringup teleop.launch
    ```

    或者

    ```bash
    rosrun RMbot_bringup rmbot_teleop
    ```

4. 启动slam

    使用gmapping算法包：

    再打开一个终端，输入命令

    ```bash
    roslaunch RMbot_slam gmapping.launch
    ```

    使用cartographer算法包

    打开一个终端，输入命令

    ```bash
    roslaunch cartographer_ros rm_demo.launch
    ```

5. 启动导航（已知地图数据）

    打开终端，输入命令

    ```bash
    roslaunch RMbot_nav amcl.launch
    ```

    再打开一个终端，输入命令

    ```bash
    roslaunch RMbot_nav move_base.launch
    ```

6. 打开可视化界面，打开终端输入命令

    ```bash
    rosrun rviz rviz
    ```

    如想要查看submaps数据类型的可视化效果，需要下载cartographer_rviz中的插件包实现可视化。其余数据均可以直接查看显示。

## 3. 远程控制

需要将主机置于同一局域网下，参考配置如下：

| 计算机名称       | 主/从机 | ip地址          | ROS_MASTER_URI                 | ROS_HOSTNAME     |
| ---------------- | ------- | --------------- | ------------------------------ | ---------------- |
| hustrm-NUC8i5BEH | 主机    | 192.168.240.212 | http://hustrm-nuc8i5beh:11311/ | hustrm-NUC8i5BEH |
| moz              | 从机    | 192.168.240.97  | http://hustrm-nuc8i5beh:11311/ | moz              |

