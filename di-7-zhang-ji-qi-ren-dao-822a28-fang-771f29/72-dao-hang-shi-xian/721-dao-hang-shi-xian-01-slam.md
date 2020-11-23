### 7.2.1 导航实现01\_SLAM建图

SLAM算法有多种，当前我们选用gmapping，下一章再介绍其他几种常用的SLAM实现。

#### 1.gmapping简介

gmapping 是ROS开源社区中较为常用且比较成熟的SLAM算法之一，gmapping可以根据移动机器人里程计数据和激光雷达数据来绘制二维的栅格地图，对应的，gmapping对硬件也有一定的要求:

* 该移动机器人可以发布里程计消息
* 机器人需要发布雷达消息\(该消息可以通过水平固定安装的雷达发布，或者也可以将深度相机消息转换成雷达消息\)

关于里程计与雷达数据，仿真环境中可以正常获取的，不再赘述，栅格地图如案例所示。

> gmapping 安装:
>
> `sudo apt-get install ros-<ROS版本>-gmapping`

#### 2.gmapping节点说明

gmapping 功能包中的核心节点是:slam\_gmapping。为了方便调用，需要先了解该节点订阅的话题、发布的话题、服务以及相关参数。

##### 2.1订阅的Topic

tf \(tf/tfMessage\)

* 用于雷达、基座与里程计之间的坐标变换消息。

scan\(sensor\_msgs/LaserScan\)

* SLAM所需的雷达信息。

##### 2.2发布的Topic

map\_metadata\(nav\_msgs/MapMetaData\)

* 地图元数据，包括地图的宽度、高度、分辨率等，该消息会固定更新。

map\(nav\_msgs/OccupancyGrid\)

* 地图栅格数据，一般会在rviz中以图形化的方式显示。

~entropy\(std\_msgs/Float64\)

* 机器人姿态分布熵估计\(值越大，不确定性越大\)。

##### 2.3服务

dynamic\_map\(nav\_msgs/GetMap\)

* 用于获取地图数据。

##### 2.4参数

~base\_frame\(string, default:"base\_link"\)

* 机器人基坐标系。

~map\_frame\(string, default:"map"\)

* 地图坐标系。

~odom\_frame\(string, default:"odom"\)

* 里程计坐标系。

~map\_update\_interval\(float, default: 5.0\)

* 地图更新频率，根据指定的值设计更新间隔。

~maxUrange\(float, default: 80.0\)

* 激光探测的最大可用范围\(超出此阈值，被截断\)。

~maxRange\(float\)

* 激光探测的最大范围。

.... 参数较多，上述是几个较为常用的参数，其他参数介绍可参考官网。

##### 2.5所需的坐标变换

雷达坐标系→基坐标系

* 一般由 robot\_state\_publisher 或 static\_transform\_publisher 发布。

基坐标系→里程计坐标系

* 一般由里程计节点发布。

##### 2.6发布的坐标变换

地图坐标系→里程计坐标系

* 地图到里程计坐标系之间的变换。

#### 3.gmapping使用

##### 3.1编写gmapping节点相关launch文件

launch文件编写可以参考 github 的演示 launch文件：[https://github.com/ros-perception/slam\_gmapping/blob/melodic-devel/gmapping/launch/slam\_gmapping\_pr2.launch](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/launch/slam_gmapping_pr2.launch)

复制并修改如下:

```xml
<launch>
<param name="use_sim_time" value="true"/>
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
      <remap from="scan" to="scan"/>
      <param name="base_frame" value="base_footprint"/><!--底盘坐标系-->
      <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="16.0"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>
<node pkg="rviz" type="rviz" name="rviz" />
<!-- 可以保存 rviz 配置并后期直接使用-->
<!--
<node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_nav_sum)/rviz/gmapping.rviz"/>
-->
</launch>
```

关键代码解释：

```xml
<remap from="scan" to="scan"/><!-- 雷达话题 -->
<param name="base_frame" value="base_footprint"/><!--底盘坐标系-->
<param name="odom_frame" value="odom"/> <!--里程计坐标系-->
```

##### 3.2执行

1.先启动 Gazebo 仿真环境\(此过程略\)

2.然后再启动地图绘制的 launch 文件:

`roslaunch 包名 launch文件名`

3.启动键盘键盘控制节点，用于控制机器人运动建图

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

4.在 rviz 中添加组件，显示栅格地图![](/assets/slam演示.PNG)最后，就可以通过键盘控制gazebo中的机器人运动，同时，在rviz中可以显示gmapping发布的栅格地图数据了，下一步，还需要将地图单独保存。

