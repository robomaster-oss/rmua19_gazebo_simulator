# rmua19_ignition_simulator

### 1.简介

rmua19_ignition_simulator是基于Ignition Gazebo的仿真环境，为RoboMaster University AI Challenge 2019中的机器人算法开发提供仿真环境，加快开发效率。目前rmua19_ignition_simulator还不完善，仅提供以下功能：


在rmua19标准机器人（rmua19_standard_robot）上增加相关传感器，并实例化不同机器人模型:
  * 搭载云台相机industrial_camera和搭载激光雷达rplidar_a2，其中相机放置有在yaw轴（对应rmua19_standard_robot1模型）和pitch轴（对应rmua19_standard_robot2模型）两种方案
  * 实例化不同颜色（红，蓝），不同编号（1号，2号）机器人模型(rmua19_standard_robot2_red1,rmua19_standard_robot2_red2,rmua19_standard_robot2_blue1,rmua19_standard_robot2_blue2)

> RoboMaster University AI Challenge 2019标准机器人机器人模型(rmua19_standard_robot)位于[rmoss_ign_resources](https://github.com/robomaster-oss/rmoss_ign_resources)

构建RoboMaster University AI Challenge 2019简易场地(models/rmua19_battlefield):
  * 只有围墙

> 注意：[Ignition Gazebo](https://github.com/ignitionrobotics/ign-gazebo)目前依然处于快速开发期，仿真功能不完善，且可能存在Bug。

### 2.使用说明

**环境配置**

ROS2和Ignition版本

* ROS2：foxy
* Ignition：Dome

```bash
# install ros-ign package
sudo apt-get install ros-foxy-ros-ign
# cd src directory of ros2 workspace 
git clone https://github.com/robomaster-oss/rmoss_interfaces
git clone https://github.com/robomaster-oss/rmoss_ign
git clone https://github.com/robomaster-oss/rmoss_ign_resources
git clone https://github.com/robomaster-oss/rmua19_ignition_simulator
# cd ros2 workspace
colcon build
```

**启动仿真环境**

```bash
ros2 launch rmua19_ignition_simulator standard_robot2_test.launch.py 
```

* 注意：需要点击ignition界面上的橙红色的`启动`按钮

![](doc/imgs/start.png)

**控制机器人移动**

```bash
ros2 run rmoss_ign_base test_chassis_cmd.py --ros-args -r __ns:=/standard_robot_red1/robot_base-p v:=0.3 -p w:=0.3
```

根据以下提示输入

```bash
This node takes keypresses from the keyboard and publishes them
as ChassisCmd messages.
---------------------------
Moving around:
        w    
   a    s    d
turn : '[' for left  ']' for right
stop : space key
---------------------------
CTRL-C to quit
```

* 底盘采用mecanum插件控制

**控制机器人云台**

```bash
ros2 run rmoss_ign_base test_gimbal_cmd.py --ros-args -r __ns:=/standard_robot_red1/robot_base
```

根据以下提示输入

```bash
This node takes keypresses from the keyboard and publishes them
as GimbalCmd messages.
---------------------------
contorl around:
        w    
   a    s    d
change  interval : '[' to decrease,  ']' to increase
---------------------------
CTRL-C to quit
```

* 云台采用位置PID控制

### 3.维护者及开源许可证

Maintainer: Zhenpeng Ge, zhenpeng.ge@qq.com

rmua19_ignition_simulator is provided under MIT License.

