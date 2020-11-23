### 8.5.3 基于rosserial\_arduino的底盘实现\_01框架搭建

**需求:**Arduino端搭建框架，主要实现速度消息的订阅、时时速度的发布。

**分析:**涉及的知识点主要是Arduino与ROS通信实现。

**实现:**

**1.arduino端代码**

```cpp
/*
 * 阶段1:搭建框架，主要实现速度消息的订阅、时时速度的发布以及各个引脚和机器人参数的定义。
 * 1.包含头文件
 * 2.编写订阅实现
 * 3.编写发布实现
 * 4.setup初始化
 * 5.loop中注意使用spinOnce()并编写测试代码
 * 
 */
#include <ros.h>
#include <geometry_msgs/Twist.h> //订阅速度消息类型
#include <geometry_msgs/Vector3.h> //发布速度消息类型,只发布线速度与角速度两个数据，使用 Vector3 即可

ros::NodeHandle nh;
//订阅
void twistCb(const geometry_msgs::Twist& twist){

}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",twistCb);

//发布
geometry_msgs::Vector3 car_vel;
ros::Publisher pub("car_vel",&car_vel);


void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop() {
  //阶段1:测试发布者代码
  delay(1000);
  car_vel.x = 1.0;
  car_vel.y = 2.0;
  pub.publish(&car_vel);

  nh.spinOnce();
}
```

**2.ROS端测试**

首先启动rosmaster:

```
roscore
```

再启动rosserial串口通信节点:

```
rosrun rosserial_python serial_node.py  /dev/ttyACM0
```

调用命令即可输出Arduino发布的消息:

```
rostopic echo /car_vel
```

一个简单的ROSArduino通信架构搭建完毕了。

