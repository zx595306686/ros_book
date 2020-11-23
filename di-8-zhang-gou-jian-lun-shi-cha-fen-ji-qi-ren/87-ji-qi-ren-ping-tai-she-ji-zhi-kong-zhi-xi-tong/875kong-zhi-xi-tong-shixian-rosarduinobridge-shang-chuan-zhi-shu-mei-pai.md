### 8.7.5控制系统实现\_rosArduinoBridge上传至树莓派

如果你已经搭建并测试通过了分布式环境，下一步，就可以将ros\_arduino\_bridge功能包上传至树莓派，并在PC端通过键盘控制小车的运动了，实现流程如下:

1. 硬件组装；
2. 系统准备；
3. 从PC端上传程序至树莓派；
4. 分别启动PC与树莓派端相关节点，并实现运动控制。

#### 1.硬件组装

将底盘Arduino与树莓派相连，将电源与树莓派相连，组装效果如下:

![](/assets/控制系统效果.PNG)

#### 2.系统准备

前面介绍过，ros\_arduino\_bridge是依赖于python-serial功能包的，请先在树莓派端安装该功能包，安装命令:

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

#### 3.程序上传

在PC端进入工作空间的src目录，调用程序上传命令:

```
scp -r ros_arduino_bridge/ 树莓派用户名@树莓派ip:~/工作空间/src
```

在树莓派端进入工作空间并编译:

```
catkin_make
```

#### 4.测试

**PC端**

先启动 ros master：

```
roscore
```

再启动键盘控制节点：

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**树莓派端**

启动 ros\_arduino\_bridge 节点:

```
roslaunch ros_arduino_python arduino.launch
```

如无异常，现在就可以在PC端通过键盘控制小车运动了，并且PC端还可以使用rviz查看小车的里程计信息。

