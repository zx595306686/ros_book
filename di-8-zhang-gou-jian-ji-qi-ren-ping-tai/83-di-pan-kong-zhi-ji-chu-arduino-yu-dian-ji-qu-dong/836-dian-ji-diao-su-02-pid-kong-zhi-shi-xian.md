### 8.3.6 电机调速02\_PID控制实现

了解了PID原理以及计算公式之后，我们可以在程序中自实现PID相关算法实现，不过，在Arduino中该算法已经被封装了，直接整合调用即可，从而提高程序的安全性与开发效率。该库是:Arduino-PID-Library，接下来通过一个案例演示该库的使用。

**需求:**通过PID控制电机转速，预期转速为 120r/m。

**实现流程:**

1. 添加Arduino-PID-Library；
2. 接线——Arduino集成电机驱动模块、电池、电机；
3. 编写Arduino程序直接调用相关实现；
4. 使用串口绘图器调试PID值。

#### 1.添加Arduino-PID-Library

首先在 GitHub 下载 PID 库: git clone [https://github.com/br3ttb/Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)

然后将该文件夹移动到 arduino 的安装目录的 libraries下: sudo cp -r Arduino-PID-Library /usr/share/arduino/libraries

还要重命名文件夹: sudo mv Arduino-PID-Library ArduinoPIDLibrary

最后重启 ArduinoIDE

#### 2.接线

需要按照8.3.2中的接线模式集成电机驱动模块、电池与电机线；

需要按照8.3.4中的接线模式集成Arduino与电机编码器。

接线效果图如下:![](/assets/PID控制接线.jpg)

#### 3.编码

PID调速中，测速是实现闭环的关键实现，所以需要复制之前的电机控制代码以及测速代码。

完整代码实现:

```cpp
/*
 * PID 调速实现:
 * 1.代码准备，复制并修改电机控制以及测速代码
 * 2.包含PID头文件
 * 3.创建PID对象
 * 4.在setup中启用自动调试
 * 5.调试并更新PWM
 * 
 */

#include <PID_v1.h> 

int DIRA = 4;
int PWMA = 5;

int motor_A = 18;//中端口是5
int motor_B = 19;//中断口是4
volatile int count = 0;//如果是正转，那么每计数一次自增1，如果是反转，那么每计数一次自减1 


void count_A(){
  //单频计数实现
  //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比
  /*if(digitalRead(motor_A) == HIGH){

    if(digitalRead(motor_B) == LOW){//A 高 B 低
      count++;  
    } else {//A 高 B 高
      count--;  
    }


  }*/

  //2倍频计数实现
  //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 2
  if(digitalRead(motor_A) == HIGH){

    if(digitalRead(motor_B) == HIGH){//A 高 B 高
      count++;  
    } else {//A 高 B 低
      count--;  
    }


  } else {

    if(digitalRead(motor_B) == LOW){//A 低 B 低
      count++;  
    } else {//A 低 B 高
      count--;  
    }  

  }

}

//与A实现类似
//4倍频计数实现
//手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 4
void count_B(){
  if(digitalRead(motor_B) == HIGH){

    if(digitalRead(motor_A) == LOW){//B 高 A 低
      count++;
    } else {//B 高 A 高
      count--;
    }


  } else {

    if(digitalRead(motor_A) == HIGH){//B 低 A 高
      count++;
    } else {//B 低 A 低
      count--;
    }

  }

}


int reducation = 60;//减速比，根据电机参数设置，比如 15 | 30 | 60
int pulse = 13; //编码器旋转一圈产生的脉冲数该值需要参考商家电机参数
int per_round = pulse * reducation * 4;//车轮旋转一圈产生的脉冲数 
long start_time = millis();//一个计算周期的开始时刻，初始值为 millis();
long interval_time = 50;//一个计算周期 50ms
double current_vel;

//获取当前转速的函数
void get_current_vel(){
  long right_now = millis();  
  long past_time = right_now - start_time;//计算逝去的时间
  if(past_time >= interval_time){//如果逝去时间大于等于一个计算周期
    //1.禁止中断
    noInterrupts();
    //2.计算转速 转速单位可以是秒，也可以是分钟... 自定义即可
    current_vel = (double)count / per_round / past_time * 1000 * 60;
    //3.重置计数器
    count = 0;
    //4.重置开始时间
    start_time = right_now;
    //5.重启中断
    interrupts();

    Serial.println(current_vel);

  }
}

//-------------------------------------PID-------------------------------------------
//创建 PID 对象
//1.当前转速 2.计算输出的pwm 3.目标转速 4.kp 5.ki 6.kd 7.当输入与目标值出现偏差时，向哪个方向控制
double pwm;//电机驱动的PWM值
double target = 120;
double kp=1.5, ki=3.0, kd=0.1;
PID pid(&current_vel,&pwm,&target,kp,ki,kd,DIRECT);

//速度更新函数
void update_vel(){
  //获取当前速度
  get_current_vel();
  pid.Compute();//计算需要输出的PWM
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,pwm);

}

void setup() {
  Serial.begin(57600);//设置波特率  
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  //两个电机驱动引脚都设置为 OUTPUT
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);

  attachInterrupt(5,count_A,CHANGE);//当电平发生改变时触发中断函数
  //四倍频统计需要为B相也添加中断
  attachInterrupt(4,count_B,CHANGE);

  pid.SetMode(AUTOMATIC);
}



void loop() {
  delay(10);
  //digitalWrite(DIRA,HIGH);
  //analogWrite(PWMA,150);
  //get_current_vel();
  update_vel();

}
```

核心代码解释:

1.包含PID头文件

```cpp
#include <PID_v1.h>
```

2.创建PID对象

```cpp
//创建 PID 对象
//1.当前转速 2.计算输出的pwm 3.目标转速 4.kp 5.ki 6.kd 7.当输入与目标值出现偏差时，向哪个方向控制
double pwm;//电机驱动的PWM值
double target = 120;
double kp=1.5, ki=3.0, kd=0.1;
PID pid(&current_vel,&pwm,&target,kp,ki,kd,DIRECT);
```

3.setup中启用PID自动控制

```cpp
pid.SetMode(AUTOMATIC);
```

4.计算输出值

```cpp
pid.Compute();
```

#### 4.调试

PID控制的最终预期结果，是要快速、精准、稳定的达成预期结果，P主要用于控制响应速度，I主要用于控制精度，D主要用于减小震荡增强系统稳定性，三者的取值是需要反复调试的，调试过程中需要查看系统的响应曲线，根据响应曲线以确定合适的PID值。

在 Arduino 中响应曲线的查看可以借助于 Serial.println\(\) 将结果输出，然后再选择菜单栏的工具下串口绘图器以图形化的方式显示响应结果:![](/assets/PID调试.png)PID调试技巧:

参数整定找最佳，从小到大顺序查

先是比例后积分，最后再专把微分加

曲线振属荡很频繁，比例度盘要放大

曲线漂浮绕大湾，比例度盘往小扳

曲线偏离回复慢，积分时间往下降

曲线波动周期长，积分时间再加长

曲线振荡频率快，先把微分降下来

动差大来波动慢。微分时间应加长

理想曲线两个波，前高后低4比1

一看二调多分析，调节质量不会低

