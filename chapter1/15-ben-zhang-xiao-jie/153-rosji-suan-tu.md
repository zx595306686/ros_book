### 1.5.3 ROS计算图

#### 1.计算图简介

前面介绍的是ROS文件结构，是磁盘上 ROS 程序的存储结构，是静态的，而 ros 程序运行之后，不同的节点之间是错综复杂的，ROS 中提供了一个实用的工具:rqt\_graph。

rqt\_graph能够创建一个显示当前系统运行情况的动态图形。ROS 分布式系统中不同进程需要进行数据交互，计算图可以以点对点的网络形式表现数据交互过程。rqt\_graph是rqt程序包中的一部分。

#### 2.计算图安装

如果前期把所有的功能包（package）都已经安装完成，则直接在终端窗口中输入

rosrun rqt\_graph rqt\_graph

如果未安装则在终端（terminal）中输入

```
$ sudo apt-get install ros-<distro>-rqt
$ sudo apt-get install ros-<distro>-rqt-common-plugins
```

请使用你的ROS版本名称（比如:kinetic、melodic、Noetic等）来替换掉&lt;distro&gt;。

例如当前版本是 Noetic,就在终端窗口中输入

```
$ sudo apt-get install ros-noetic-rqt
$ sudo apt-get install ros-noetic-rqt-common-plugins
```

#### 3.计算图演示

接下来以 ROS 内置的小乌龟案例来演示计算图

首先，按照前面所示，运行案例

然后，启动新终端，键入: rqt\_graph 或 rosrun rqt\_graph rqt\_graph，可以看到类似下图的网络拓扑图，该图可以显示不同节点之间的关系。![](/assets/计算图.PNG)

