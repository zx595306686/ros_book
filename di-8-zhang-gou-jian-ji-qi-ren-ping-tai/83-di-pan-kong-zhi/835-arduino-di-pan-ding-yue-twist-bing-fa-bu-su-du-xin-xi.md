```cpp
/*
 * 需求：实现机器人底盘控制，订阅ROS端发布的 /cmd_vel 消息，
 * 并通过 PID 调速转换成 PWM 数据控制电机运动，还需要时时发布
 * 机器人底盘的速度信息
 * 
 * 
 * 实现阶段1: 框架搭建，能够订阅 /cmd_vel 消息，能够发布时时速度信息
 * 1.包含头文件
 *   ros.h
 *   geometry_msgs/Twist.h  ---> /cmd_vel
 *   geometry_msgs/Vector3.h --> 小车速度信息
 *   
 * 2.创建 NodeHandle
 * 3.创建订阅者对象以及回调函数
 * 4.创建发布者对象(先生成伪数据)
 * 5.setup 中执行初始化
 * 6.loop 中发布数据
 * 
 * 实现阶段2: 将订阅到的 /cmd_vel 消息转换成机器人底盘左右轮的速度信息,进而再转变为左右轮的转速信息
 * 速度转换公式:
 *   左轮速度 = 线速度 - 角速度 * 轮胎间距 / 2;
 *   右轮速度 = 线速度 + 角速度 * 轮胎间距 / 2;
 * 转速转换公式:
 *   左轮转速 = 左轮速度 / 轮直径 / PI
 *   右轮转速 = 右轮速度 / 轮直径 / PI
 *   
 *   
 * 实现阶段3: 电机基本控制实现(暂不集成)
 * 控制车辆前进，后退与停止
 * 注意，先确定车的前进与后退方向，电机的转向要与车辆保持一致
 * 1.设置转向引脚与使能引脚
 * 2.分别实现左右轮的前进、后退与停止函数
 * 
 * 实现阶段4: 后续需要通过PID调速来控制电机运动以达成预期速度，而PID调速依赖于测速，当前先完成测速功能
 * 1.设置编码器相关引脚
 * 2.编写中断函数统计脉冲数
 * 3.取某个时间区间内的中断数并计算转速
 * 
 * 
 * 实现阶段5: PID 调速，根据 PID 算法调整电机转速到预期
 * 0.下载 PID 库
 * 1.包含头文件
 * 2.创建 PID 对象并设置参数
 * 3.设置为自动调速 
 * 4.输出调速后的 PWM 值
 * 
 * 
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <PID_v1.h>


//参数
double left2right = 0.2; //左右轮胎间距
double wheel_d = 0.065; //轮直径
double wheel_r = wheel_d / 2; //轮半径
double left_vel_target,right_vel_target;

//变量
double left_rotate_target, right_rotate_target; //左右轮的目标转速

ros::NodeHandle nh;
//订阅
void twistCb(const geometry_msgs::Twist& twist){
  left_vel_target = twist.linear.x - twist.angular.z * left2right / 2;
  right_vel_target = twist.linear.x + twist.angular.z * left2right / 2;
  left_rotate_target = left_vel_target / wheel_d / PI * 60;
  right_rotate_target = right_vel_target / wheel_d / PI * 60;
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",twistCb);

//发布
geometry_msgs::Vector3 car_vel;
ros::Publisher pub("car_vel",&car_vel);


//电机控制
//左电机-------------------------------------------------------------------
int DIRA = 4;//转向引脚
int PWMA = 5;//使能引脚


//右电机-------------------------------------------------------------------
int DIRB = 7;//转向引脚
int PWMB = 6;//使能引脚


//左前进
void left_forward(int pwm){
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,pwm);
} 
//左后退
void left_back(int pwm){
  digitalWrite(DIRA,LOW);
  analogWrite(PWMA,pwm);
} 
//左停止
void left_stop(){
  analogWrite(PWMA,0);
}

//右前进
void right_forward(int pwm){
  digitalWrite(DIRB,LOW);
  analogWrite(PWMB,pwm);
} 
//右后退
void right_back(int pwm){
  digitalWrite(DIRB,HIGH);
  analogWrite(PWMB,pwm);
} 
//右停止
void right_stop(){
  analogWrite(PWMB,0);  
}

//测速实现-------------------------------------------------------------------------
//编码器相关引脚(测试实现所依赖)
// 左轮编码器 B 相连接引脚 18(中断口5)，左轮编码器 A 相连接引脚 19(中断口4)
// 右轮编码器 B 相连接引脚 20(中断口3)，右轮编码器 A 相连接引脚 21(中断口2) 
int left_A = 19; //4
int left_B = 18; //5
int right_A = 21; //2
int right_B = 20; //3

volatile int left_A_count;  //左轮 A 相脉冲计数器
volatile int right_A_count; //右轮 A 相脉冲计数器

long last_time = millis(); //用于统计上次的结束时刻

//转速统计
double left_rotate_current;
double right_rotate_current;

int per_circle = 390;//输出轴一圈的脉冲数

//左电机脉冲统计
void left_A_sum(){

  if(digitalRead(left_B) == LOW){
    left_A_count++; 
  } else {
    left_A_count--; 
  }

}
//右电机脉冲统计
void right_A_sum(){
  if(digitalRead(right_B) == HIGH){
    right_A_count++;  
  } else {
    right_A_count--;  
  }
}

//电机脉冲统计重置
void rest_sum(){
  left_A_count = 0;
  right_A_count = 0;  
}



//获取当前速度信息
void get_current_vel(){
  //car_vel.x 与  car_vel.y
  //轮速度 = 轮转速 / 60 * 轮直径 * PI;
  //线速度 = (左轮速度 + 右轮速度) / 2
  //角速度 = (右轮速度 - 左轮速度) / 轮胎间距
  double left_vel = (left_rotate_current / 60 * wheel_d * PI);
  double right_vel = (right_rotate_current / 60 * wheel_d * PI);
  car_vel.x = (left_vel + right_vel) / 2;
  car_vel.y = (right_vel - left_vel) / left2right;
}

void get_current_rotate(){
  long right_now = millis();
  long paste_time = right_now - last_time;
  if(paste_time >= 100){//如果逝去时间 >= 100 毫秒
    //终止中断
    noInterrupts();
    //统计转速
    //左轮转速
    left_rotate_current = (double)left_A_count / 390 / paste_time * 1000 * 60;
    //右轮转速
    right_rotate_current = (double)right_A_count / 390 / paste_time * 1000 * 60;

    //获取速度信息，然后发布
    get_current_vel();
    pub.publish(&car_vel);

    //计数器归零
    rest_sum();
    //结束时刻重置
    last_time = millis();
    //重启中断
    interrupts();

  }
}

//调速实现-------------------------------------------------------------------------
double kp=0.5, ki=0.4, kd=0.01;
double left_pwm_out=0;
double left_rotate_current_abs = 0;
PID left_pid(&left_rotate_current_abs,&left_pwm_out,&left_rotate_target,kp,ki,kd,DIRECT);
double right_pwm_out=0;
double right_rotate_current_abs = 0;
PID right_pid(&right_rotate_current_abs,&right_pwm_out,&right_rotate_target,kp,ki,kd,DIRECT);


void update_rotate(){
  left_rotate_current_abs = abs(left_rotate_current);
  left_rotate_target = abs(left_rotate_target);
  if(left_vel_target == 0){
    //左轮停止
    left_stop();  
  } else if(left_vel_target > 0){
    left_pid.Compute();
    left_forward(left_pwm_out);
  } else { 
    left_pid.Compute();
    left_back(left_pwm_out); 
    //left_stop();  
  }

  right_rotate_current_abs = abs(right_rotate_current);
  right_rotate_target = abs(right_rotate_target);
  if(right_vel_target == 0){
    //右轮停止
    right_stop();  
  } else if(right_vel_target > 0){
    right_pid.Compute();
    right_forward(right_pwm_out);
  } else {

    right_pid.Compute();
    right_back(right_pwm_out);  
  }

}



//初始化引脚-----------------------------------------------------------------------
void initMode(){
  //电机运动控制
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  pinMode(PWMA,OUTPUT);


  //电机编码器引脚设置
  pinMode(left_A,INPUT);
  pinMode(left_B,INPUT);
  pinMode(right_A,INPUT);
  pinMode(right_B,INPUT);


  //中断设置
  attachInterrupt(4,left_A_sum,FALLING);//左轮A相统计脉冲数
  attachInterrupt(2,right_A_sum,FALLING);//右轮A相统计脉冲数

}

void setup() {
  Serial.begin(57600);
  initMode();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  //设置为自动调速
  left_pid.SetMode(AUTOMATIC);
  right_pid.SetMode(AUTOMATIC);
}

void loop() {
  //获取当前电机速度
  get_current_rotate();
  //调速
  update_rotate();
  nh.spinOnce();
  delay(10);
}
```



