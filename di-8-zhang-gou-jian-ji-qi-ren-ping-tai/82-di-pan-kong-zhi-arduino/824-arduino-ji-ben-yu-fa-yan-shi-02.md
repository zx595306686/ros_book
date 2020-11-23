### 8.2.4 arduino 基本语法演示02

#### **1.**数字IO操作

需求:控制LED灯开关，在一个循环周期内前两秒使LED灯处于点亮状态，后两秒关闭LED灯

实现:

```cpp
/*
 * 控制LED灯开关，在一个循环周期内前两秒使LED灯处于点亮状态，后两秒关闭LED灯
 * 1.setup 中设置引脚为输出模式
 * 2.loop 中向引脚输出高电压，休眠 2000 毫秒后，再输出低电压，再休眠 2000 毫秒
 * 
 */
int led = 13;
void setup() {
  Serial.begin(57600);
  pinMode(led,OUTPUT);

}

void loop() {

  digitalWrite(led,HIGH);//输出高电压  
  delay(2000);

  digitalWrite(led,LOW);//输出低电压
  delay(2000); 

}
```

#### 2.模拟IO操作

需求:控制LED灯亮度

原理:在1中LED灯只有关闭或开启两种状态，是无法控制 LED 灯亮度，如果要实现此功能，那么需要借助于 PWM\(Pulse width modulation 脉冲宽度调制\)技术，通过设置占空比为LED间歇性供电，PWM 的取值范围 \[0,255\]。

实现:

```cpp
/*
 * 需求:控制LED灯亮度
 * 实现:
 *  1.setup 中设置 led 灯的引脚为输出模式
 *  2.设置不同的 PWM 并输出
 * 
 */
int led = 13;
int l1 = 255;
int l2 = 50;
int l3 = 0;
void setup() {
  pinMode(led,OUTPUT);
}

void loop() {
  analogWrite(led,l1);
  delay(2000);
  analogWrite(led,l2);
  delay(2000);
  analogWrite(led,l3);
  delay(2000);

}
```

运行结果:在一个周期内LED灯亮度递减直至熄灭



