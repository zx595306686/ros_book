### 8.6.1 基于ros\_arduino\_bridge的底盘实现\_概述

#### 1.ros\_arduino\_bridge 简介

该功能包包含Arduino库和用来控制Arduino的ROS驱动包，它旨在成为在ROS下运行Arduino控制的机器人的完整解决方案。

其中当前主要关注的是:功能包集中一个兼容不同驱动的机器人的基本控制器（base controller），它可以接收ROS Twist类型的消息，可以发布里程数据到ROS端。

**特点:**

* 可以直接支持ping声呐和Sharp红外线传感器

* 也可以从通用的模拟和数字信号的传感器读取数据

* 可以控制数字信号的输出

* 支持PWM伺服机

* 如果使用所要求的硬件的话，可以配置基本的控制

* 如果你的Arduino编程基础好的话，并且具有python基础的话，你就可以很自由的改动代码来满足你的硬件要求

**注意：**

* Robogaia Mega Encoder shield 仅适用于Arduino Mega，
* 板上编码计数器\(ARDUINO\_ENC\_COUNTER\)目前仅支持Arduino Uno
* 上面非硬性规定，有一定的编程基础，你也可以按需更改

**系统要求:**

ros\_arduino\_bridge 依赖于 python-serial 功能包，请提前安装此包，安装命令:

```
$ sudo apt-get install python-serial
```

或

```
$ sudo pip install --upgrade pyserial

```

或

```
$ sudo easy_install -U pyserial
```

**下载:**

进入ROS工作空间的src目录,输入命令:

```
git clone https://github.com/hbrobotics/ros_arduino_bridge.git
```

#### 2.ros\_arduino\_bridge 架构

文件结构说明

```
├── ros_arduino_bridge                      # metapackage (元功能包)
│   ├── CMakeLists.txt
│   └── package.xml
├── ros_arduino_firmware                    #固件包，更新到Arduino
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── libraries                       #库目录
│           ├── MegaRobogaiaPololu          #针对Pololu电机控制器，MegaRobogaia编码器的头文件定义
│           │   ├── commands.h              #定义命令头文件
│           │   ├── diff_controller.h       #差分轮PID控制头文件
│           │   ├── MegaRobogaiaPololu.ino  #PID实现文件
│           │   ├── sensors.h               #传感器相关实现，超声波测距，Ping函数
│           │   └── servos.h                #伺服器头文件
│           └── ROSArduinoBridge            #Arduino相关库定义
│               ├── commands.h              #定义命令
│               ├── diff_controller.h       #差分轮PID控制头文件
│               ├── encoder_driver.h        #编码器驱动头文件
│               ├── encoder_driver.ino      #编码器驱动实现, 读取编码器数据，重置编码器等
│               ├── motor_driver.h          #电机驱动头文件
│               ├── motor_driver.ino        #电机驱动实现，初始化控制器，设置速度
│               ├── ROSArduinoBridge.ino    #核心功能实现，程序入口
│               ├── sensors.h               #传感器头文件及实现
│               ├── servos.h                #伺服器头文件，定义插脚，类
│               └── servos.ino              #伺服器实现
├── ros_arduino_msgs                        #消息定义包
│   ├── CMakeLists.txt
│   ├── msg                                 #定义消息
│   │   ├── AnalogFloat.msg                 #定义模拟IO浮点消息
│   │   ├── Analog.msg                      #定义模拟IO数字消息
│   │   ├── ArduinoConstants.msg            #定义常量消息
│   │   ├── Digital.msg                     #定义数字IO消息
│   │   └── SensorState.msg                 #定义传感器状态消息
│   ├── package.xml
│   └── srv                                 #定义服务
│       ├── AnalogRead.srv                  #模拟IO输入
│       ├── AnalogWrite.srv                 #模拟IO输出
│       ├── DigitalRead.srv                 #数字IO输入
│       ├── DigitalSetDirection.srv　　　　 #数字IO设置方向
│       ├── DigitalWrite.srv                #数字IO输入
│       ├── ServoRead.srv                   #伺服电机输入
│       └── ServoWrite.srv                  #伺服电机输出
└── ros_arduino_python                      #ROS相关的Python包，用于上位机，树莓派等开发板或电脑等。
    ├── CMakeLists.txt
    ├── config                              #配置目录
    │   └── arduino_params.yaml             #定义相关参数，端口，rate，PID，sensors等默认参数。由arduino.launch调用
    ├── launch
    │   └── arduino.launch                  #启动文件
    ├── nodes
    │   └── arduino_node.py                 #python文件,实际处理节点，由arduino.launch调用，即可单独调用。
    ├── package.xml
    ├── setup.py
    └── src                                 #Python类包目录
        └── ros_arduino_python
            ├── arduino_driver.py           #Arduino驱动类
            ├── arduino_sensors.py          #Arduino传感器类
            ├── base_controller.py          #基本控制类，订阅cmd_vel话题，发布odom话题
            └── __init__.py                 #类包默认空文件
```

上述目录结构虽然复杂，但是关注的只有两大部分:

* ros\_arduino\_bridge/ros\_arduino\_firmware/src/libraries/ROSArduinoBridge
* ros\_arduino\_bridge/ros\_arduino\_python/config/arduino\_params.yaml

前者是Arduino端的固件包实现，需要修改并上传至Arduino电路板；

后者是ROS端的一个配置文件，相关驱动已经封装完毕，我们只需要修改配置信息即可。

整体而言，借助于 ros\_arduino\_bridge可以大大提高我们的开发效率。

#### 3.案例实现

基于ros\_arduino\_bridge的底盘实现具体步骤如下:

* 了解并修改Arduino端程序主入口ROSArduinoBridge.ino 文件；
* Arduino端添加编码器驱动；
* Arduino端添加电机驱动模块；

* Arduino端实现PID调试；

* ROS端修改配置文件。



