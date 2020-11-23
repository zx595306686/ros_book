## 7.2 导航实现

本节内容主要介绍导航的完整性实现，旨在掌握机器人导航的基本流程，该章涉及的主要内容如下:

* SLAM\(选用较为流行的gmapping演示\)建图
* 地图服务\(可以保存和重现地图\)
* 机器人定位

* 路径规划

上述三部分内容介绍完毕，还会对功能进一步集成简化机器人SLAM建图。

---

**准备工作**

请先安装相关的ROS功能包:

安装 gmapping 包\(用于构建地图\):`sudo apt-get install ros-<ROS版本>-gmapping`

安装地图服务包\(用于保存地图\):`sudo apt-get install ros-<ROS版本>-map-server`

安装 navigation 包\(用于定位以及路径规划\):`sudo apt-get install ros-<ROS版本>-navigation`

新建功能包，并导入依赖: gmapping map\_server amcl move\_base

**补充**

如果使用的时ROS最新版本noetic，可能还未包含gmapping功能包，可以自行安装:

首先安装:openslam-gmapping

```
sudo apt install ros-noetic-openslam-gmapping
```

然后下载gmapping源码，进入工作空间的src目录，下载功能包源码:

```
git clone https://github.com/ros-perception/slam_gmapping.git
```

最后，编译源码。

建议: 如果noetic没有相关功能包，请将ROS版本降级为 melodic。

