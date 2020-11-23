## 4.2 ROS节点运行管理launch文件

关于 launch 文件的使用我们已经不陌生了，在第一章内容中，就曾经介绍到:

> 一个程序中可能需要启动多个节点，比如:ROS 内置的小乌龟案例，如果要控制乌龟运动，要启动多个窗口，分别启动 roscore、乌龟界面节点、键盘控制节点。如果每次都调用 rosrun 逐一启动，显然效率低下，如何优化?

采用的优化策略便是使用roslaunch 命令集合 launch 文件启动管理节点，并且在后续教程中，也多次使用到了 launch 文件。

---

#### **概念**

launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数。

#### **作用**

简化节点的配置与启动，提高ROS程序的启动效率。

#### **使用**

以 turtlesim 为例演示

1.新建launch文件

在功能包下添加 launch目录, 目录下新建 xxxx.launch 文件，编辑 launch 文件

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node"     name="myTurtle" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key"  name="myTurtleContro" output="screen" />
</launch>
```

2.调用 launch 文件

```
roslaunch 包名 xxx.launch
```

**注意:**roslaunch 命令执行launch文件时，首先会判断是否启动了 roscore,如果启动了，则不再启动，否则，会自动调用 roscore

**PS:**本节主要介绍launch文件的使用语法，launch 文件中的标签，以及不同标签的一些常用属性。

