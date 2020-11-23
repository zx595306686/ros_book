## 7.2 导航实现01\_SLAM

在此，将使用 SLAM 中较为成熟的 gmapping 来绘制全局地图，该流程实现，主要有如下步骤:

1. 安装 ROS 中相关的功能包

2. 绘制地图

3. 保存地图

#### 1.安装功能包

安装 gmapping 包\(用于构建地图\):`sudo apt-get install ros-<ROS版本>-gmapping`

安装地图服务包\(用于保存地图\):`sudo apt-get install ros-<ROS版本>-map-server`

创建功能包，导入依赖: gmapping、map\_server 包

#### 2.绘制地图

当前功能包下新建 launch 文件,内容如下:

```xml
<launch>
<node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <!--<param name="base_frame" value="base_footer"/> -->
    <param name="base_frame" value="base_footprint"/>
    <!--底盘坐标系-->
    <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
    <param name="map_update_interval" value="1.0"/> <!--更新时间(s)，每多久更新一次地图，不是频率-->
    <param name="maxUrange" value="20.0"/> <!--激光雷达最大可用距离，在此之外的数据截断不用-->
    <param name="maxRange" value="25.0"/> <!--激光雷达最大距离-->
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="200"/>
    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>
    <param name="linearUpdate" value="0.5"/>
    <param name="angularUpdate" value="0.436"/>
    <param name="temporalUpdate" value="-1.0"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="particles" value="80"/>
    <param name="xmin" value="-25.0"/>
    <param name="ymin" value="-25.0"/>
    <param name="xmax" value="25.0"/>
    <param name="ymax" value="25.0"/>
    <param name="delta" value="0.05"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>
    <remap from="scan" to="scan"/>
</node>
<node pkg="rviz" type="rviz" name="rviz" />
<!-- 可以保存 rviz 配置并后期直接使用-->
<!--
<node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_nav_sum)/rviz/gmapping.rviz"/>
-->
</launch>
```

#### 3.保存地图

新建 launch 文件用于保存构建的地图:

```xml
<launch>
    <arg name="filename" value="$(find my_nav_sum)/map/nav" />
    <node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```

#### 4.执行

1.先启动 Gazebo 仿真环境\(此过程略\)

2.然后再启动地图绘制的 launch 文件:

`roslaunch 包名 launch文件名`

3.启动键盘键盘控制节点，控制机器人运动建图

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

4.调用 map\_server 文件，保存绘制的地图

`roslaunch 包名 launch文件名`

5.运行结束之后，会在指定目录下生成两个文件: xxx.pgm 和 xxx.yaml

前者是一张图片资源\(绘制的地图\)，后者是关于地图的描述文件

#### 5.补充

如果使用的ROS版本还未包含gmapping功能包，可以自行安装:

首先安装:openslam-gmapping

```
sudo apt install ros-noetic-openslam-gmapping
```

然后下载gmapping源码，进入工作空间的src目录，下载功能包源码:

```
git clone https://github.com/ros-perception/slam_gmapping.git
```

最后，编译源码

