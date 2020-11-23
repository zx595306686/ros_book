### 8.2.5 arduino 基本语法演示03

需求:调用 millis\(\) 函数获取程序当前已经执行的时间，调用delay\(\)函数实现休眠

实现:

```cpp
/*
 * 需求:调用 millis() 函数获取程序当前已经执行的时间，调用delay()函数实现休眠
 * 
 * 1.setup 中设置波特率
 * 2.loop 中使用delay休眠，使用millis获取程序执行时间并输出
 * 
 */

unsigned long past_time;

void setup() {
  Serial.begin(57600);
}

void loop() {
  delay(2000);//休眠 2 秒

  past_time  = millis();
  Serial.println(past_time);  
}
```

通过串口监视器查看输出结果。

![](/assets/arduino时间.PNG)

