### 8.3.2 Arduino 与 ROS 交互\_发布

**需求:**实现Arduino与ROS端交互，Arduino端创建发布节点，话题自定义，发布字符串消息"Hello World"，ROS端订阅并打印订阅到的消息

**实现:**

**1.Arduino端编写发布实现**

```cpp
/*
 * 创建发布者对象：
 *  话题：chatter
 *  消息：Hello World
 *  
 * 实现：
 * 1.导入相关头文件 ros.h std_msgs/String.h
 * 2.创建发布者对象
 * 3.setup 中初始化节点
 * 4.loop中发布消息
 * 
 */

#include <ros.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher pub("chatter",&msg);
char str[15]="Hello World!!!";

void setup() {
  nh.initNode();//初始化 ros 节点
  nh.advertise(pub); //发布
}

void loop() {
  msg.data = str;
  pub.publish(&msg);
  nh.spinOnce();
  delay(1000);
}
```

**2.ROS端启动串口通信**

首先启动rosmaster:

```
roscore
```

再启动rosserial串口通信节点:

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

启动串口通信中，不能通过Arduino IDE 向Arduino电路板上传程序，必须先关闭ros的串口通信节点才能上传，否则会导致上传失败。

**3.ROS端订阅发布的消息**

```
rostopic echo /chatter
```

结果:ROS端输出 HelloWorld

