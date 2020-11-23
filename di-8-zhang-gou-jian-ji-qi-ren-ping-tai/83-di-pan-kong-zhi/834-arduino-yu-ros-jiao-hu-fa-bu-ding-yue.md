### 8.3.4 Arduino 与 ROS 交互\_发布订阅

**需求:**发布与订阅集成，发布文本，订阅空消息切换LED状态

**实现:**

**1.arduino端实现**

```cpp
/*
 * 订阅且发布消息：
 * Arduino 端，发布HelloWorld
 *            订阅消息并切换 LED 状态
 * 
 * 1.导入头文件
 * 2.创建 NodeHandle
 * 3.创建订阅者与发布者对象
 * 4.setup 中实现初始化
 * 5.loop 中发布消息，并 spinOnce
 * 
 * 
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

//发布
std_msgs::String msg;
char str[10] = "hello ros";
ros::Publisher pub("chatter",&msg);

//订阅
void msgCb(const std_msgs::Empty &msg){
  digitalWrite(13,HIGH-digitalRead(13));
}
ros::Subscriber<std_msgs::Empty> sub("toggle",&msgCb);

void setup() {
  pinMode(13,OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop() {
  msg.data = str;
  pub.publish(&msg);
  nh.spinOnce();
  delay(1000);
}
```

其他操作参考前两节内容

