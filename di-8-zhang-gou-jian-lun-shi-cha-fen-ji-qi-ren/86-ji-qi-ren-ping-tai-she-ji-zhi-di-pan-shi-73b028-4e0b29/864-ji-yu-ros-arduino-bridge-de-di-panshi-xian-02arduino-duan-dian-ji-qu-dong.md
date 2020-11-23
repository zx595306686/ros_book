### 8.6.4 基于ros\_arduino\_bridge的底盘实现\_03Arduino端电机驱动

自定义电机驱动的实现与上一节的编码器驱动流程类似:

1. ROSArduinoBridge.ino 中需要注释之前的电机驱动，添加自定义电机驱动；
2. motor\_driver.h 中设置左右电机引脚；
3. motor\_driver.ino 中实现初始化与速度设置函数；
4. 测试

#### 1.定义电机驱动

ROSArduinoBridge.ino需要添加电机宏定义,代码如下:

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
   /* 使用自定义的编码器驱动 */
   #define ARDUINO_MY_COUNTER

   /* L298 Motor driver*/
   //#define L298_MOTOR_DRIVER
   //使用自定义的L298P电机驱动
   #define L298P_MOTOR_DRIVER
#endif
```

#### 2.修改motor\_driver.h文件

修改后内容如下:

```cpp
/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined L298P_MOTOR_DRIVER
  #define DIRA 4
  #define PWMA 5
  #define DIRB 7
  #define PWMB 6
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
```

#### 3.修改motor\_driver.ino 文件

主要添加内容如下:

```cpp
#elif defined L298P_MOTOR_DRIVER
  void initMotorController() {
    pinMode(DIRA,OUTPUT);
    pinMode(PWMA,OUTPUT);
    pinMode(DIRB,OUTPUT);
    pinMode(PWMB,OUTPUT);
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255){
      spd = 255;
    }

    if (i == LEFT) { 
      if(reverse == 0) { 
        digitalWrite(DIRA,LOW);
        analogWrite(PWMA,spd);
      }else if (reverse == 1) { 
        digitalWrite(DIRA,HIGH);
        analogWrite(PWMA,spd);
      }
    }else{
      if(reverse == 0) { 
        digitalWrite(DIRB,HIGH);
        analogWrite(PWMB,spd);
      }
      else if (reverse == 1) { 
        digitalWrite(DIRB,LOW);
        analogWrite(PWMB,spd);
      }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
```

#### 4.测试

编译并上传程序，打开串口监视器，然后输入命令，命令格式为: m 左电机转速 右电机转速，比如: m 20 20，左右电机开始转动

> 注意: m 命令命令后的参数1与参数2为电机转速，确切的讲，按照 ros\_arduino\_bridge的默认算法，参数1和参数2的时间单位为1/30 秒，也即: m 20 20 时，左右电机设置的转速都是 20 \* 30 = 600 \(r/m\)。



