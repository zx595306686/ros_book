### 8.5.4 基于rosserial\_arduino的底盘实现\_02电机转向控制

**需求:**Arduino端实现电机转向的控制并测试；

**分析:**需要设置左右电机的使能以及转向引脚。

**实现:**

```cpp
/*
 * 阶段1:搭建框架，主要实现速度消息的订阅、时时速度的发布以及各个引脚和机器人参数的定义。
 * 1.包含头文件
 * 2.编写订阅实现
 * 3.编写发布实现
 * 4.setup初始化
 * 5.loop中注意使用spinOnce()并编写测试代码
 * 
 * 阶段2:实现电机转向的控制并测试
 * 1.设置左右电机相关引脚
 * 2.分别编写左右电机的前进、后退以及停止函数
 * 3.在 loop 中测试
 * 
 * 
 */
#include <ros.h>
#include <geometry_msgs/Twist.h> //订阅速度消息类型
#include <geometry_msgs/Vector3.h> //发布速度消息类型,只发布线速度与角速度两个数据，使用 Vector3 即可

//--------------------------------------转向控制---------------------------------------
int DIRA = 4;//左轮转向
int PWMA = 5;//左轮使能

int DIRB = 7;//右轮转向
int PWMB = 6;//右轮使能

void left_forward(int pwm){
  digitalWrite(DIRA,LOW);
  analogWrite(PWMA,pwm);
}
void left_stop(){
  digitalWrite(DIRA,LOW);
  analogWrite(PWMA,0);
}
void left_back(int pwm){
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,pwm);
}

void right_forward(int pwm){
  digitalWrite(DIRB,HIGH);
  analogWrite(PWMB,pwm);
}
void right_stop(){
  digitalWrite(DIRB,LOW);
  analogWrite(PWMB,0);
}
void right_back(int pwm){
  digitalWrite(DIRB,LOW);
  analogWrite(PWMB,pwm);
}


//--------------------------------------订阅发布---------------------------------------
ros::NodeHandle nh;
//订阅
void twistCb(const geometry_msgs::Twist& twist){

}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",twistCb);

//发布
geometry_msgs::Vector3 car_vel;
ros::Publisher pub("car_vel",&car_vel);

//--------------------------------------引脚设置---------------------------------------
void initMode(){
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  pinMode(PWMB,OUTPUT);
}
void setup() {
  Serial.begin(57600);

  //将引脚初始化封装进单独函数
  initMode();

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop() {

  nh.spinOnce();
//阶段1:测试发布者代码
//  delay(1000);
//  car_vel.x = 1.0;
//  car_vel.y = 2.0;
//  pub.publish(&car_vel);
//阶段2:测试转向
/*
  left_forward(100);  
  right_forward(100); 
  delay(3000);

  left_stop();  
  right_stop(); 
  delay(3000);

  left_back(100);  
  right_back(100); 
  delay(3000);

  left_stop();  
  right_stop(); 
  delay(3000);
*/

}
```



