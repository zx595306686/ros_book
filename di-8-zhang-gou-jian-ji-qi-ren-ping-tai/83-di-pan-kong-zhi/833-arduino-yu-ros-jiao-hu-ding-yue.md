### 8.3.3 Arduino 与 ROS 交互\_订阅

**需求:**实现Arduino与ROS端交互，ROS端发布空消息，Arduino端订阅空消息，并切换LED灯状态。

**实现:**

**1.Arduino端编写订阅实现**

```cpp
/*
 * 订阅ROS端发布的消息，并切换LED状态
 * 1.导入头文件
 * 2.创建订阅者对象，以及处理函数
 * 3.setup执行初始化
 * 4.loop中执行spinonce
 * 
 */
#include <ros.h>
#include <std_msgs/Empty.h>


int led = 13;
ros::NodeHandle nh;
void msgCb(const std_msgs::Empty& msg){
  digitalWrite(led,HIGH - digitalRead(led));
}
ros::Subscriber<std_msgs::Empty> sub("toggle_led",&msgCb);

void setup() {
  pinMode(led,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  nh.spinOnce();
  delay(1000);
}
```

**2.ROS端启动串口**

首先启动rosmaster:

```
roscore
```

再启动rosserial串口通信节点:

```
rosrun rosserial_python serial_node.py  /dev/ttyACM0
```

**3.ROS端发布空消息**

```
rostopic pu-r 0.5 /toggle_led std_msgs/Empty "{}"
```

结果: Arduino端LED等闪烁

