### 8.5.6 基于rosserial\_arduino的底盘实现\_04调速

**需求:**将订阅的ROS端发布的速度消息转换成左右电机的转速，并通过PID调速调整转速到预期。

**分析:**当前有左右两个电机，需要创建两个PID对象分别调速，每个PID对象调速需要的参数主要有输入\(当前转速\)、目标值、输出\(PWM\)、KP、KI、KD，输入\(当前转速\)上一节已经实现，输出定义变量接收即可，KP、KI、KD是需要调试的变量，核心是目标值的设置，从ROS端订阅的是底盘的预期速度消息，可以通过固定公式将底盘的速度消息转换成左右电机的速度，公式如下:

* 左轮速度 = 线速度 - 角速度 \* 轮胎间距 / 2;
* 右轮速度 = 线速度 + 角速度 \* 轮胎间距 / 2;

当然，车轮速度可以转换成电机转速,公式:电机转速 = 车轮速度 / PI / 车轮直径;

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
 * 阶段4:实现电机调速
 * 1.包含头文件
 * 2.分别创建左右轮的PID对象，其中还需要定义相关参数
 *   输入值: 电机当前转速(已实现)
 *   目标值: 需要订阅者计算产生
 *   输出值: 需要定义(左右电机的输出PWM)
 *   KP、KI、KD 需要调试
 * 3.在 setup 中设置PID对象的控制方式为自动  
 * 4.编写速度更新函数
 *   先获取当前速度，然后根据不同的目标速度分支(速度等于0 大于0 或小于0)处理
 * 5.loop中调用速度更新函数
 * 
 */
#include <ros.h>
#include <geometry_msgs/Twist.h> //订阅速度消息类型
#include <geometry_msgs/Vector3.h> //发布速度消息类型,只发布线速度与角速度两个数据，使用 Vector3 即可
#include <PID_v1.h>

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

//----------------------------发布---------------------------
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


//--------------------------------------PID调速---------------------------------------
//-----订阅-----
//订阅
double left_target;//左电机目标转速
double right_target;//右电机目标转速
void twistCb(const geometry_msgs::Twist& twist){
  double left_vel = twist.linear.x - twist.angular.z * left2right / 2; //速度 m/s
  double right_vel = twist.linear.x + twist.angular.z * left2right / 2; //速度 m/s
  left_target = left_vel / PI / wheel_d * 60;// r/m
  right_target = right_vel / PI / wheel_d * 60;// r/m
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",twistCb);


//左电机
double left_pid_in; //左电机输入
double left_pid_target;//左电机目标
double left_pid_pwm; //左电机输出
double left_p = 1.6,left_i = 5.0,left_d = 0.0;
PID left_pid(&left_pid_in,&left_pid_pwm,&left_pid_target,left_p,left_i,left_d,DIRECT);

//右电机

double right_pid_in;
double right_pid_target;
double right_pid_pwm;
double right_p = 1.6,right_i = 5.0 ,right_d = 0.0;
PID right_pid(&right_pid_in,&right_pid_pwm,&right_pid_target,right_p,right_i,right_d,DIRECT);

void update_vel(){
  get_current_vel();//获取当前转速
  //组织参数
  left_pid_in = abs(left_current_vel);
  left_pid_target = abs(left_target);
  left_pid.Compute();
  if(left_target > 0){
    left_forward(left_pid_pwm);
  } else if(left_target < 0){
    left_back(left_pid_pwm);
  } else {
    left_stop();  
  }
  //Serial.println(left_current_vel);//调试 pid

  right_pid_in = abs(right_current_vel);
  right_pid_target = abs(right_target);
  right_pid.Compute();
  if(right_target > 0){
    right_forward(right_pid_pwm);
  } else if(right_target < 0){
    right_back(right_pid_pwm);  
  } else {
    right_stop();  
  }
  //Serial.println(right_current_vel);//调试 pid
}

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
ros::NodeHandle nh;
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

  //PID设置为自动
  left_pid.SetMode(AUTOMATIC);
  right_pid.SetMode(AUTOMATIC);
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

//阶段4:调速实现
  delay(10);
  update_vel();


}
```



