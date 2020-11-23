### 8.3.4 电机测速02\_实现

**需求:**统计并输出电机转速。

**思路:**先统计单位时间内以单频或2倍频或4倍频的方式统计脉冲数，再除以一圈对应的脉冲数，最后再除以时间所得即为电机转速。

**核心:**计数时，需要在A相或B相的上升沿或下降沿触发时，实现计数，在此需要使用中断引脚与中断函数。

Arduino Mega 2560 的中断引脚:2 \(interrupt 0\), 3 \(interrupt 1\),18 \(interrupt 5\), 19 \(interrupt 4\), 20 \(interrupt 3\), 21 \(interrupt 2\)

**实现流程:**

1. 接线:将Arduino与电机驱动模块集成，使用杜邦线将电机编码器连接到Arduino的相关引脚；
2. 编写Arduino程序先实现脉冲数统计；
3. 编写Arduino程序再实现转速计算相关实现；
4. 上传到Arduino并测试。

#### 1.接线

先将Arduino与电机驱动模块集成\(以实现\)；

然后使用杜邦线\(公对母\)将**编码器5V**与**编码器GND**分别连接到Arduino或电机驱动板的5V和GND；

最后使用杜邦线\(公对母\)将编码器A相和编码器B相分别连接中断引脚\(比如:A相连接18，B相连接19\)。

**注意:**当前只是测速案例，没必要必须连接电池以及为电机供电，精简期间，当前只连接编码器。

接线效果图如下:![](/assets/编码器接线1.jpg)![](/assets/编码器接线2.png)

#### 2.编码实现脉冲统计

核心知识点:**attachInterrupt\(\)函数**

> **描述:**当发生外部中断时，调用一个指定函数。当中断发生时，该函数会取代正在执行的程序。大多数的Arduino板有两个外部中断：0（数字引脚2）和1（数字引脚3）。
>
> arduino Mege有四个外部中断：数字2（引脚21），3（引脚20），4（引脚19），5（引脚18）。
>
> **语法:**attachInterrupt\(interrupt, function, mode\)
>
> interrupt：中断引脚数
>
> function：中断发生时调用的函数，此函数必须不带参数和不返回任何值。该函数有时被称为中断服务程序。
>
> mode：定义何时发生中断以下四个contstants预定有效值：
>
> * LOW 当引脚为低电平时，触发中断
>
> * CHANGE 当引脚电平发生改变时，触发中断
>
> * RISING 当引脚由低电平变为高电平时，触发中断
>
> * FALLING 当引脚由高电平变为低电平时，触发中断.
>
> **返回:**无
>
> **注意事项:**当中断函数发生时，delay\(\)和millis\(\)的数值将不会继续变化。当中断发生时，串口收到的数据可能会丢失。你应该声明一个变量来在未发生中断时储存变量。

**代码:**

```cpp
/*
 * 测速实现:
 *  阶段1:脉冲数统计
 *  阶段2:速度计算
 * 
 * 阶段1:
 *  1.定义所使用的中断引脚,以及计数器(使用 volatile 修饰)
 *  2.setup 中设置波特率，将引脚设置为输入模式
 *  3.使用 attachInterupt() 函数为引脚添加中断出发时机以及中断函数
 *  4.中断函数编写计算算法，并打印
 *    A.单频统计只需要统计单相上升沿或下降沿
 *    B.2倍频统计需要统计单相的上升沿和下降沿
 *    C.4倍频统计需要统计两相的上升沿和下降沿
 *  5.上传并查看结果
 *  
 * 
 */
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

void setup() {
  Serial.begin(57600);//设置波特率  
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  attachInterrupt(5,count_A,CHANGE);//当电平发生改变时触发中断函数
  //四倍频统计需要为B相也添加中断
  attachInterrupt(4,count_B,CHANGE);
}


void loop() {
  //测试计数器输出
  delay(3000);
  Serial.println(count);

}
```

#### 3.转速计算

思路:需要定义一个开始时间\(用于记录每个测速周期的开始时刻\)，还需要定义一个时间区间\(比如50毫秒\)，时时获取当前时刻，当当前时刻 - 上传结束时刻 &gt;= 时间区间时，就获取当前计数并根据测速公式计算时时速度，计算完毕，计数器归零，重置开始时间

核心知识点:当使用中断函数中的变量时，需要先禁止中断**noInterrupts\(\)**，调用完毕，再重启中断**interrupts\(\)**

**代码\(核心\):**

2中代码除了 loop 实现，无需修改。

```cpp
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

void loop() {
  //测试计数器输出
  //delay(3000);
  //Serial.println(count);

  delay(10);
  get_current_vel();

}
```

#### 4.测试

将代码上传至Arduino，打开出口监视器，手动旋转电机，可以查看到转速信息。

