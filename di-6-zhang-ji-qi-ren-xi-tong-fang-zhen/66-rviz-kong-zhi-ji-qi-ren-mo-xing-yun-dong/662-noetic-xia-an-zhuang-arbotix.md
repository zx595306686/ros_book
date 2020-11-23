### 6.5.2 Noetic下安装Arbotix

**注意:**

在 ROS 最新版本 noetic 中，使用`sudo apt-get install ros-noetic-arbotix`安装 arbotix 失败。

错误提示:`无法定位软件包 ros-noetic-arbotix`

原因:`arbotix`未更新。

那么此种情况下，必须使用源码安装，流程如下:

#### 1.准备

准备1: 安装 git

准备2: 安装 pip3

#### 2.arbotix 异常\_serial

编译 arbotix 后，执行:`rosrun arbotix_python arbotix_driver`会抛出异常:

```
Traceback (most recent call last):
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/bin/arbotix_driver", line 36, in <module>
    from arbotix_python.arbotix import ArbotiX, ArbotiXException
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/src/arbotix_python/arbotix.py", line 36, in <module>
    import serial, time, sys, _thread
ModuleNotFoundError: No module named 'serial'
```

需要安装 serial 模块，安装命令:`pip3 install serial`

#### 3.arbotix 源码修改\_print

安装 serial 包后，继续编译执行，抛出如下异常:

```
Traceback (most recent call last):
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/bin/arbotix_driver", line 36, in <module>
    from arbotix_python.arbotix import ArbotiX, ArbotiXException
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/src/arbotix_python/arbotix.py", line 94
    print e
          ^
SyntaxError: Missing parentheses in call to 'print'. Did you mean print(e)?
```

这是因为 nortic 使用的 Python3,不兼容 Python2 根据提示，修改源码,将 print e 修改为 print\(e\)

后续还会遇到多个此类异常，也是直接根据提示修改源码

#### 4.arbotix 源码修改\_thread

print 修正完毕后，继续编译执行，会遇到以下问题:

```
Traceback (most recent call last):
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/bin/arbotix_driver", line 36, in <module>
    from arbotix_python.arbotix import ArbotiX, ArbotiXException
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/src/arbotix_python/arbotix.py", line 33, in <module>
    import serial, time, sys, thread
ModuleNotFoundError: No module named 'thread'
```

这是因为，在 Python3 中 thread 模块被 \_thread 替代，请将源码中的 thread 相关导包以及调用统统替换为 \_thread

#### 5.arbotix 源码修改\_导包操作

\_thread 修改完毕，继续编译执行，抛出以下异常:

```
Traceback (most recent call last):
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/bin/arbotix_driver", line 36, in <module>
    from arbotix_python.arbotix import ArbotiX, ArbotiXException
  File "/home/xuzuo/ros_urdf/src/arbotix_ros/arbotix_python/src/arbotix_python/arbotix.py", line 34, in <module>
    from ax12 import *
ModuleNotFoundError: No module named 'ax12'
```

需要设置环境变量，具体操作: 在 arbotix.py 导包之前，加入如下代码:

```
import sys
sys.path.append("/xxx/yyy/自定义的工作空间路径/src/arbotix_ros/arbotix_python/src/arbotix_python")
```

后续还会 抛出 print 相关异常，直接修改即可

至此，arbotix 在 noetic 中的基本使用环境搭建完毕。

