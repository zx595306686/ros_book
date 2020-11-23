### 8.2.3 arduino 基本语法演示

通信操作

#### 1.通信实现01

需求: 通过串口，由 arduino 向计算机发送数据

实现:

```cpp
/*
 * 需求:通过串口，由 arduino 向计算机发送数据
 * 实现:
 *  1.setup中设置波特率
 *  2.setup 或 loop 中使用 Serial.print 或 Serial.println() 发送数据
 * 
 * 
 * 
 */
void setup() {
  Serial.begin(57600);
  Serial.println("setup");
}

void loop() {
  delay(3000);
  Serial.print("loop");
  Serial.print("  ");
  Serial.println("hello");
}
```

![](/assets/arduino通信1.PNG)

#### 2.通信实现02

需求: 通过串口，由计算机向Arduino发送数据

实现:

```cpp
/*
 * 需求:通过串口，由计算机向 arduino 发送数据
 * 实现:
 *  1.setup中设置波特率
 *  2.loop 中接收发送的数据，并打印
 * 
 * 
 * 
 */
char num;
void setup() {
  Serial.begin(57600);
}

void loop() {
  if(Serial.available() > 0){
    num = Serial.read();
    Serial.print("I accept:");
    Serial.println(num);  
  }
}
```

![](/assets/arduino通信2.PNG)

