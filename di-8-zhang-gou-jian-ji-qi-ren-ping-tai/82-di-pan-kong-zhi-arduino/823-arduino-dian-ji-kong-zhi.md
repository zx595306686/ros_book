### 8.2.3 arduino 电机转动控制

说明:通过arduino可以控制电机转动\(正传+反转\)以及转速，控制电机转动，但是不能直接通过 arduino 控制实现，arduino 的输出的电信号偏弱不足以驱动电机，需要额外的电机驱动板来放大arduino的电信号进而控制电机

电机驱动板型号众多，在此我们选用 L298P，使用方便\(先将Arduino与电机驱动板集成\)

---

需求:使用 arduino 控制电机运动

1.连线:使用杜邦线连接

PWMA\(使能\) 与 电机+相连，DIRA\(转向\)与电机-相连

2.编码

```cpp
int DIRA=4; //转向引脚
int PWMA=5; //使能引脚

void setup(){
  Serial.begin(57600);
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
}

void loop(){
  //前进
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,60);
  delay(3000);
  
  //停止
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,0);
  delay(3000);
  
  //后退
  digitalWrite(DIRA,LOW);
  analogWrite(PWMA,60);
  delay(3000);
  
  //停止
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,0);
  delay(3000);
}
```

---



