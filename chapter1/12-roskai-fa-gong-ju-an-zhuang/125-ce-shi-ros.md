### 1.2.5 测试 ROS

ROS 内置了一些小程序，可以通过运行这些小程序以检测 ROS 环境是否可以正常运行

1. 首先启动三个命令行\(ctrl + alt + T\)

2. 命令行1键入:**roscore**

3. 命令行2键入:**rosrun turtlesim turtlesim\_node**\(此时会弹出图形化界面\)

4. 命令行3键入:**rosrun turtlesim turtle\_teleop\_key**\(在3中可以通过上下左右控制2中乌龟的运动\)

最终结果如下所示:

![](/assets/01ROS环境测试.png "01ROS环境测试")注意：光标必须聚焦在键盘控制窗口，否则无法控制乌龟运动。

