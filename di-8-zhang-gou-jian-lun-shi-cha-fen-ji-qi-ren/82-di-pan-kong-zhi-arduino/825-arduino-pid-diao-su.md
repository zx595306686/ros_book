### 8.2.5 arduino PID 调速

#### 1.添加Arduino-PID-Library

首先在 GitHub 下载 PID 库: git clone [https://github.com/br3ttb/Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)

然后将该文件夹移动到 arduino 的安装目录的 libraries下: sudo cp -r Arduino-PID-Library /usr/share/arduino/libraries

还要重命名文件夹: sudo mv Arduino-PID-Library ArduinoPIDLibrary

最后重启 ArduinoIDE

#### 2.演示案例

```cpp
/*

  实现 PID 调速

*/
#include <PID_v1.h>

int motor_A = 19;
volatile long motor_A_count = 0;
double start_time=millis();
double right_now;
int DIRA=4; //转向引脚
int PWMA=5; //使能引脚

double current_vel;

void get_A_count(){
  motor_A_count++;
//  Serial.println(motor_A_count);
}




void get_current_vel(){
  right_now = millis();
  double past_time = right_now - start_time;
  if(past_time >= 100){
    // Serial.println("----------------");
    // 取消中断 
    noInterrupts();
    // 计算
    current_vel = (double)motor_A_count * 1000 * 60 / 390 / past_time;
    Serial.println(current_vel);
    motor_A_count = 0;
    // 开始中断
    interrupts(); 
    start_time=millis();
  }
}

//设置调速所需参数

//创建 PID 对象
//1.当前转速 2.计算输出的pwm 3.目标转速 4.kp 5.ki 6.kd 7.当输入与目标值出现偏差时，向哪个方向控制
double pwm=0;
double target = 300;
double kp=0.5, ki=0.4, kd=0.01;
PID pid(&current_vel,&pwm,&target,kp,ki,kd,DIRECT);

//调速
void update_vel(){
  pid.Compute();
  //Serial.println(pwm);
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,pwm);
}

void setup(){
  Serial.begin(57600);
  pinMode(motor_A,INPUT);
  attachInterrupt(4,get_A_count,FALLING);
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pid.SetMode(AUTOMATIC);
}

void loop(){
  delay(10);
  get_current_vel();
  update_vel();
}
```

核心代码:

创建pid对象

```cpp
double pwm=0;
double target = 300;
double kp=0.5, ki=0.4, kd=0.01;
PID pid(&current_vel,&pwm,&target,kp,ki,kd,DIRECT);
```

设置为自动调速

```cpp
pid.SetMode(AUTOMATIC);
```

调速

```cpp
pid.Compute();
```

#### 3.PID值调试

PID调试技巧:

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

![](/assets/PID调试.png)

