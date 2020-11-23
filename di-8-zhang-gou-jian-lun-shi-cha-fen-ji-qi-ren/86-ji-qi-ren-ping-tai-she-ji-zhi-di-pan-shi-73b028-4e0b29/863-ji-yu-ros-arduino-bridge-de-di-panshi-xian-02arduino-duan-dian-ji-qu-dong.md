### 8.6.3 基于ros\_arduino\_bridge的底盘实现\_02Arduino端编码器驱动

测速是整个PID闭环控制中的必须环节，我们必须修改代码适配当前AB相编码器，虽然需要重写功能，但是测速部分内容已经封装，只需要实现编码器计数即可，大致实现流程如下:

1. ROSArduinoBridge.ino 中需要注释之前的编码器驱动，添加自定义编码器驱动；
2. encoder\_driver.h 中设置编码器引脚并声明初始化函数以及中断函数；
3. encoder\_driver.ino 中实现编码器计数以及重置函数；
4. ROSArduinoBridge.ino 中 setup 函数调用编码器初始化函数。
5. 测试

#### 1.定义编码器驱动

ROSArduinoBridge.ino需要添加编码器宏定义,代码如下:

```cpp
#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* Encoders directly attached to Arduino board */
   //#define ARDUINO_ENC_COUNTER
   #define ARDUINO_MY_COUNTER

   /* L298 Motor driver*/
   //#define L298_MOTOR_DRIVER

   #define L298P_MOTOR_DRIVER
#endif
```

先去除 \#define L298P\_MOTOR\_DRIVER 的注释，否则后续编译会抛出异常。

#### 2.修改encoder\_driver.h文件

修改后内容如下:

```cpp
/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */


#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3

  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined ARDUINO_MY_COUNTER
  #define LEFT_A 18
  #define LEFT_B 19
  #define RIGHT_A 20
  #define RIGHT_B 21
  void initEncoders();
  void leftEncoderEvent();
  void rightEncoderEvent();
#endif

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
```

#### 3.修改encoder\_driver.ino 文件

主要添加内容如下:

```cpp
#elif defined ARDUINO_MY_COUNTER
  long left_count = 0;//左轮计数器
  long right_count = 0;//右轮计数器
  void initEncoders(){
    pinMode(LEFT_A, INPUT);
    pinMode(LEFT_B, INPUT);
    pinMode(RIGHT_A, INPUT);
    pinMode(RIGHT_B, INPUT);

    attachInterrupt(5, leftEncoderEventA, CHANGE);
    attachInterrupt(4, leftEncoderEventB, CHANGE);
    attachInterrupt(3, rightEncoderEventA, CHANGE);
    attachInterrupt(2, rightEncoderEventB, CHANGE);
  }
  void leftEncoderEventA(){
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
  void leftEncoderEventB(){
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
  void rightEncoderEventA(){
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

  void rightEncoderEventB(){
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
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_count;
    else return right_count;    // It's just because my right encoder get reverse value so if yours is normal, don't add "-"
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_count=0L;
      return;
    } else {
      right_count=0L;
      return;
    }
  }
```

#### 4.ROSArduinoBridge.ino 实现初始化

setup 添加语句:initEncoders\(\);

```cpp
void setup() {
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);

    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);

    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);

    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  initEncoders();//关键----------------------------------------------------------------------
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif



}
```

#### 5.测试

编译并上传程序，打开串口监视器，然后旋转车轮，在串口监视器中录入 e 即可查看左右编码器计数，录入命令 r 可以重置计数。

