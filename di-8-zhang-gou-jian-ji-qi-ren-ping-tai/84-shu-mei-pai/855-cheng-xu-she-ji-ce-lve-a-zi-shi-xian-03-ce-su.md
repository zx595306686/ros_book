### 8.5.5 基于rosserial\_arduino的底盘实现\_03测速

**需求:**实现左右电机测速，并根据左右电机速度计算出底盘速度信息并发布。

**分析:**测速实现之前已有介绍，问题的核心是左右电机速度转换成底盘速度的调用公式，公式如下:

* 底盘线速度 = \(左轮速度 + 右轮速度\) / 2;
* 底盘角速度 = \(右轮速度 - 左轮速度\) / 左右轮间距;

当然，车轮转速可以通过转速获取,公式:车轮速度 = 转速 \* PI \* 车轮直径。

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
 * 阶段3:实现电机测速
 * 1.设置编码器引脚
 * 2.添加中断函数
 * 3.计算左右电机转速，并生成底盘速度信息发布
 * 
 * 
 */
#include <ros.h>
#include <geometry_msgs/Twist.h> //订阅速度消息类型
#include <geometry_msgs/Vector3.h> //发布速度消息类型,只发布线速度与角速度两个数据，使用 Vector3 即可

//--------------------------------------转向控制---------------------------------------
//1.设置引脚，并在initMode中设置引脚模式
int DIRA = 4;//左轮转向
int PWMA = 5;//左轮使能

int DIRB = 7;//右轮转向
int PWMB = 6;//右轮使能

//2.编写转向控制函数
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

//--------------------------------------电机测速---------------------------------------
//1.定义引脚以及相关变量，并在initMode中设置引脚模式 2.setup 中为引脚添加中断函数
int LEFT_A = 18;//5
int LEFT_B = 19;//4
int RIGHT_A = 20;//3
int RIGHT_B = 21;//2

int left_count = 0;//左轮计数器
int right_count = 0;//右轮计数器

//3.编写中断函数
void left_A_count(){
  if(digitalRead(LEFT_A) == HIGH){
    if(digitalRead(LEFT_B) == LOW){
      left_count++;  
    } else {
      left_count--;  
    }  
  } else {
    if(digitalRead(LEFT_B) == HIGH){
      left_count++;  
    } else {
      left_count--;  
    }  
  }
}
void left_B_count(){
  if(digitalRead(LEFT_B) == HIGH){
    if(digitalRead(LEFT_A) == HIGH){
      left_count++;  
    } else {
      left_count--;  
    }  
  } else {
    if(digitalRead(LEFT_A) == LOW){
      left_count++;  
    } else {
      left_count--;  
    }  
  }
}
void right_A_count(){
  if(digitalRead(RIGHT_A) == HIGH){
    if(digitalRead(RIGHT_B) == HIGH){
      right_count++;  
    } else {
      right_count--;  
    }  
  } else {
    if(digitalRead(RIGHT_B) == LOW){
      right_count++;  
    } else {
      right_count--;  
    }  
  }
}
void right_B_count(){
  if(digitalRead(RIGHT_B) == HIGH){
    if(digitalRead(RIGHT_A) == LOW){
      right_count++;  
    } else {
      right_count--;  
    }  
  } else {
    if(digitalRead(RIGHT_A) == HIGH){
      right_count++;  
    } else {
      right_count--;  
    }  
  }
}
//4.编写测速函数
//测速需要定义的变量
int reducation = 60;//减速比，根据电机参数设置，比如 15 | 30 | 60
int pulse = 13; //编码器旋转一圈产生的脉冲数该值需要参考商家电机参数
int per_round = pulse * reducation * 4;//4倍频测速下，车轮旋转一圈产生的脉冲计数
long start_time = millis();//一个计算周期的开始时刻，初始值为 millis();
long interval_time = 50;//一个计算周期 50ms
double left_current_vel;//左轮当前转速 单位 r/m
double right_current_vel;//右轮当前转速 单位 r/m
double wheel_d = 0.065;//车轮直径，用于计算轮子速度
double left2right = 0.2;//左右轮间距，用于计算角速度

//----------发布-------
geometry_msgs::Vector3 car_vel;
ros::Publisher pub("car_vel",&car_vel);

void get_current_vel(){
  long right_now = millis();
  long past_time = right_now - start_time;
  if(past_time >= interval_time){
    noInterrupts();//禁止中断  
    //计算速度
    left_current_vel = (double)left_count / per_round / past_time * 1000 * 60;// r/m
    right_current_vel = (double)right_count / per_round / past_time * 1000 * 60;// r/m
    double left_vel = left_current_vel * PI * wheel_d / 60;//速度转换 m/s
    double right_vel = right_current_vel * PI * wheel_d / 60;//速度转换 m/s
    //测试
//    Serial.print(left_current_vel);
//    Serial.print("  ");
//    Serial.println(right_current_vel);
    //计算底盘速度并发布
    double liner = (left_vel + right_vel) / 2;
    double angular = (right_vel - left_vel) / left2right;
    car_vel.x = liner;
    car_vel.y = angular;
    pub.publish(&car_vel);
    start_time = right_now;//时间重置
    //计数器归0
    left_count = 0;
    right_count = 0;
    interrupts();//重启中断
  }
}

//--------------------------------------订阅发布---------------------------------------
ros::NodeHandle nh;
//订阅
void twistCb(const geometry_msgs::Twist& twist){

}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",twistCb);



//--------------------------------------引脚设置---------------------------------------
void initMode(){
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  pinMode(PWMB,OUTPUT);

  //编码器
  pinMode(LEFT_A,INPUT);
  pinMode(LEFT_B,INPUT);
  pinMode(RIGHT_A,INPUT);
  pinMode(RIGHT_B,INPUT);
}
void setup() {
  Serial.begin(57600);

  //将引脚初始化封装进单独函数
  initMode();

  //添加中断函数
  attachInterrupt(5,left_A_count,CHANGE);
  attachInterrupt(4,left_B_count,CHANGE);
  attachInterrupt(3,right_A_count,CHANGE);
  attachInterrupt(2,right_B_count,CHANGE);

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
//阶段3:测试编码器计数(不调用get_current_vel的前提下)
//  delay(3000);
//  Serial.print(left_count);
//  Serial.print("  ");
//  Serial.println(right_count);
  //测速函数测试
//  delay(10);
//  get_current_vel();


}
```



