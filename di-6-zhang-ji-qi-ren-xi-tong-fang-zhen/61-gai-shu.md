## 6.1 概述

机器人操作系统学习、开发与测试过程中，会遇到诸多问题，比如:

> 场景1:机器人一般价格不菲，学习ROS要购买一台机器人吗？
>
> 场景2:机器人与之交互的外界环境具有多样性，如何实现复杂的环境设计？
>
> 场景3:测试时，直接将未经验证的程序部署到实体机器人运行，安全吗？
>
> ...

在诸如此类的场景中，ROS中的**仿真**就显得尤为重要了。

---

#### 1.**概念**

**机器人系统仿真：**是通过计算机对实体机器人系统进行模拟的技术，在 ROS 中，仿真实现涉及的内容主要有三:对机器人建模\(URDF\)、创建仿真环境\(Gazebo\)以及感知环境\(Rviz\)等系统性实现。

#### 2.**作用**

##### 2.1仿真优势:

仿真在机器人系统研发过程中占有举足轻重的地位，在研发与测试中较之于实体机器人实现，仿真有如下几点的显著优势:

1.**低成本:**当前机器人成本居高不下，动辄几十万，仿真可以大大降低成本，减小风险

2.**高效:**搭建的环境更为多样且灵活，可以提高测试效率以及测试覆盖率

3.**高安全性:**仿真环境下，无需考虑耗损问题

##### 2.2仿真缺陷:

机器人在仿真环境与实际环境下的表现差异较大，换言之，仿真并不能完全做到模拟真实的物理世界，存在一些"失真"的情况，原因:

1.仿真器所使用的物理引擎目前还不能够完全精确模拟真实世界的物理情况

2.仿真器构建的是关节驱动器（电机&齿轮箱）、传感器与信号通信的绝对理想情况，目前不支持模拟实际硬件缺陷或者一些临界状态等情形

#### 3.相关组件

##### 3.1URDF

**URDF**是 Unified Robot Description Format 的首字母缩写，直译为**统一\(标准化\)机器人描述格式**，可以以一种 XML 的方式描述机器人的部分结构，比如底盘、摄像头、激光雷达、机械臂以及不同关节的自由度.....,该文件可以被 C++ 内置的解释器转换成可视化的机器人模型，是 ROS 中实现机器人仿真的重要组件

##### 3.2rviz

RViz 是 ROS Visualization Tool 的首字母缩写，直译为**ROS的三维可视化工具**。它的主要目的是以三维方式显示ROS消息，可以将 数据进行可视化表达。例如:可以显示机器人模型，可以无需编程就能表达激光测距仪（LRF）传感器中的传感 器到障碍物的距离，RealSense、Kinect或Xtion等三维距离传感器的点云数据（PCD， Point Cloud Data），从相机获取的图像值等

以“ros- \[ROS\_DISTRO\] -desktop-full”命令安装ROS时，RViz会默认被安装。

运行使用命令`rviz`或`rosrun rviz rviz`

_**如果rviz没有安装，请调用如下命令自行安装:**_

```
sudo apt-get install ros-[ROS_DISTRO]-rviz
```

##### 3.3gazebo

Gazebo是一款3D动态模拟器，用于显示机器人模型并创建仿真环境,能够在复杂的室内和室外环境中准确有效地模拟机器人群。与游戏引擎提供高保真度的视觉模拟类似，Gazebo提供高保真度的物理模拟，其提供一整套传感器模型，以及对用户和程序非常友好的交互方式。

以“ros- \[ROS\_DISTRO\] -desktop-full”命令安装ROS时，gzebo会默认被安装。

运行使用命令`gazebo`或`rosrun gazebo_ros gazebo`

**注意1:**_**在 Ubuntu20.04 与 ROS Noetic 环境下，gazebo 启动异常以及解决**_

* **问题1:**VMware: vmw\_ioctl\_command error Invalid argument\(无效的参数\)

  **解决:**

  `echo "export SVGA_VGPU10=0" >> ~/.bashrc`

  `source .bashrc`

* **问题2:**\[Err\] \[REST.cc:205\] Error in REST request

  **解决:**`sudo gedit ~/.ignition/fuel/config.yaml`

  然后将`url : https://api.ignitionfuel.org`使用 \# 注释

  再添加`url: https://api.ignitionrobotics.org`

* **问题3:**启动时抛出异常:`[gazebo-2] process has died [pid xxx, exit code 255, cmd.....`

  **解决:**`killall gzserver`和`killall gzclient`

**注意2:**_**如果 gazebo没有安装，请自行安装:**_

1.添加源:

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" 
    >
     /etc/apt/sources.list.d/gazebo-stable.list'

```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

2.安装：

```
sudo apt update
```

```
sudo apt install gazebo11 
sudo apt install libgazebo11-dev
```

---

课程说明:

机器人的系统仿真是一种集成实现，主要包含三部分:

* URDF 用于创建机器人模型

* Gzebo 用于搭建仿真环境

* Rviz 图形化的显示机器人各种传感器感知到的环境信息

三者应用中，只是创建 URDF 意义不大，一般需要结合 Gazebo 或 Rviz 使用，在 Gazebo 或 Rviz 中可以将 URDF 文件解析为图形化的机器人模型，一般的使用组合为:

* 如果非仿真环境，那么使用 URDF 结合 Rviz 直接显示感知的真实环境信息

* 如果是仿真环境，那么需要使用 URDF 结合 Gazebo 搭建仿真环境，并结合 Rviz 显示感知的虚拟环境信息

后续课程安排:

* 先介绍 URDF 与 Rviz 集成使用，在 Rviz 中只是显示机器人模型，主要用于学习 URDF 语法

* 再介绍 URDF 与 Gazebo 集成，主要学习 URDF 仿真相关语法以及仿真环境搭建

* 最后集成 URDF 与 Gazebo 与 Rviz，实现综合应用



