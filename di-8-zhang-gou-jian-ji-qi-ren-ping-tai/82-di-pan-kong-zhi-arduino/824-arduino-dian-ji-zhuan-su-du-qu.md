### 8.2.4 arduino 电机转速读取

需求:通过编码器读取电机的转速信息\(ROS中需要时时获取速度信息\)

使用的电机必须保证安装有编码器

---

1.统计编码器脉冲数实现\(单相\)

```cpp
int motor_A = 19;
int motor_A_count = 0;
void get_A_count(){
  motor_A_count++;
  Serial.println(motor_A_count);
}


void setup(){
  Serial.begin(57600);
  pinMode(motor_A,INPUT);
  attachInterrupt(4,get_A_count,FALLING);
}

void loop(){

}
```

2.实现转速统计

```cpp
/*
 * 
 * 需求:4倍频测速实现
 * 
 * 核心:
 *  1.为两个引脚设置输入模式
 *  2.为两个引脚添加对应的中断
 *  3.两个引脚的中断中针对正反转各计数两次
 * 
 * 
 */


int motor_A = 19; //4
int motor_B = 18; //5



volatile long motor_count = 0;
double start_time=millis();
double right_now;

void get_A_count(){
 /*
  * 以 A 相为参考:
  * 
  * 只记录 motor_A == HIGH 那么是单倍频计数(if实现)
  * 这种情况下:
  *   车轮旋转一圈的计数: 脉冲数 * 减速比 (13 * 20)
  * 
  * 记录 motor_A == HIGH 与 motor_A == LOW 是双倍频计数(if else)
  * 这种情况下:
  *   车轮旋转一周的计数: 脉冲数 * 减速比 * 2 (13 * 30 * 2)
  *   
  * 以 B 相为参考同理
  * 
  * AB 相结合，可以以 4 倍频计数
  *   车轮旋转一周的计数: 脉冲数 * 减速比 * 4 (13 * 30 * 4)
  * 
  */
 if(digitalRead(motor_A) == HIGH ){
  if(digitalRead(motor_B) == HIGH){
      motor_count++;
  } else {
      motor_count--;  
  }
 } else {
  if(digitalRead(motor_B) == LOW){
      motor_count++;
  } else {
      motor_count--;  
  }

 }
 //Serial.println(motor_count);
}

void get_B_count(){
  if(digitalRead(motor_B) == HIGH){
    if(digitalRead(motor_A) == LOW){
      motor_count++;  
    } else {
      motor_count--;  
    }    
  } else {
    if(digitalRead(motor_A) == HIGH){
      motor_count++;  
    } else {
      motor_count--;  
    }    

  }
}


void setup(){
  Serial.begin(57600);
  pinMode(motor_A,INPUT);
  pinMode(motor_B,INPUT);
  attachInterrupt(4,get_A_count,CHANGE);
  attachInterrupt(5,get_B_count,CHANGE);
}

void get_current_vel(){
  right_now = millis();
  double past_time = right_now - start_time;
  if(past_time >= 100){
    // Serial.println("----------------");
    // 取消中断
    noInterrupts();
    // 计算
    double vel = (double)motor_count / 1560 / past_time * 1000;
    Serial.println(vel);
    //Serial.println(motor_count);
    motor_count = 0;
    // 开始中断
    interrupts();

    start_time=millis();
  }
}

void loop(){
  delay(10);
  get_current_vel();
}
```



