### 8.4.1 Arduino与ROS通信\_rosserial简介

ROS是进程（也称为_Nodes_）的分布式框架，在ROS体系中，可以很方便的实现不同ROS设备的通信\(实现方式可参考4.7\)，但是也可能会遇到ROS设备与非ROS设备通信的情况，比如:

> 在我们设计的机器人平台中，控制系统安装了ROS环境，而驱动系统是基于Arduino，Arduino端并没有也不支持安装ROS，而二者实现需要实现相互发布与订阅功能，怎么实现？

当然需要通信的非ROS设备不仅是Arduino，还可能是windows、嵌入式Linux、STM32...，在ROS中针对此类问题已经提供了相关的解决方案:rosserial。

---

#### 概念 {#A.28RECOMMENDED.29_Installing_Binaries_on_the_ROS_workstation}

rosserial是用于ROS设备与非ROS设备通信的一种协议，非ROS设备虽然没有安装ROS环境，但是借助于rosserial可以通过串口或网络轻松的实现与ROS设备诸如话题与服务的通信。

rosserial的架构主要由服务器与客户端两部分组成，服务器是运行在ROS设备上的，而客户端则是运行在非ROS设备上的，后续将以Arduino为例演示rosserial的环境搭建以及使用。

#### 安装

##### 1.ROS端软件安装

您可以通过运行以下命令为Arduino安装rosserial：

```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```

将indigo替换为您要从中安装的发行版的名称：例如hydro、kinetic、melodic。

##### 2.Arduino端软件安装

前面的安装步骤创建了必要的库，现在下面的代码将创建ros\_lib文件夹，Arduino构建环境需要此文件夹才能使Arduino程序与ROS交互。

在下面的步骤中，&lt;sketchbook&gt;是Linux Arduino环境保存草图的目录。通常，这是您主目录中的一个名为_Sketchbook_或_Arduino_的目录。例如cd〜/ Arduino / libraries 或者，您可以安装到Windows Arduino环境。

groovy源（catkin）的Ros\_lib安装说明与早期版本（rosbuild）或二进制发行版不同。确保您在上面选择了正确的构建系统以查看适当的说明-catkin用于常规的源构建，否则为rosbuild。

**注意：**您必须删除library / ros\_lib（如果存在），以便重新生成，因为它的存在会导致错误。“ rosrun rosserial\_arduino make\_libraries.py”创建ros\_lib目录。

```
  cd <sketchbook>/libraries
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
```

#### 局限性

虽然通过rosserial可以方便的实现非ROS设备与ROS设备的通信，但是其也存在局限性:

* 默认情况下，客户端发布者和订阅者的数量限制在25个之内；
* 默认情况下，通信时序列化和反序列化缓冲区的大小限制为512字节，即ROS消息的大小必须小于512字节。



