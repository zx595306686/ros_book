### 7.2.3 导航实现03\_定位

所谓定位就是推算机器人自身在全局地图中的位置，当然，SLAM中也包含定位算法实现，不过SLAM的定位是用于构建全局地图的，是属于导航开始之前的阶段，而当前定位是用于导航中，导航中，机器人需要按照设定的路线运动，通过定位可以判断机器人的实际轨迹是否符合预期。在ROS的导航功能包集navigation中提供了 acml 功能包，用于实现导航中的机器人定位。

#### 1.amcl简介

AMCL\(adaptive Monte Carlo Localization\) 是用于2D移动机器人的概率定位系统，它实现了自适应（或KLD采样）蒙特卡洛定位方法，可以根据已有地图使用粒子滤波器推算机器人位置。

> amcl安装\(amcl已经被集成到了navigation包\):
>
> `sudo apt-get install ros-<ROS版本>-navigation`

#### 2.amcl节点说明

amcl 功能包中的核心节点是:amcl。为了方便调用，需要先了解该节点订阅的话题、发布的话题、服务以及相关参数。

##### 3.1订阅的Topic

scan\(sensor\_msgs/LaserScan\)

* 激光雷达数据。

tf\(tf/tfMessage\)

* 坐标变换消息。

initialpose\(geometry\_msgs/PoseWithCovarianceStamped\)

* 用来初始化粒子滤波器的均值和协方差。

map\(nav\_msgs/OccupancyGrid\)

* 获取地图数据。

##### 3.2发布的Topic

amcl\_pose\(geometry\_msgs/PoseWithCovarianceStamped\)

* 机器人在地图中的位姿估计。

particlecloud\(geometry\_msgs/PoseArray\)

* 位姿估计集合，rviz中可以被 PoseArray 订阅然后图形化显示机器人的位姿估计集合。

tf\(tf/tfMessage\)

* 发布从 odom 到 map 的转换。

##### 3.3服务

global\_localization\(std\_srvs/Empty\)

* 初始化全局定位的服务。

request\_nomotion\_update\(std\_srvs/Empty\)

* 手动执行更新和发布更新的粒子的服务。

set\_map\(nav\_msgs/SetMap\)

* 手动设置新地图和姿态的服务。

##### 3.4调用的服务

static\_map\(nav\_msgs/GetMap\)

* 调用此服务获取地图数据。

##### 3.5参数

~odom\_model\_type\(string, default:"diff"\)

* 里程计模型选择: "diff","omni","diff-corrected","omni-corrected" \(diff 两轮差分、omni 全向轮\)

~odom\_frame\_id\(string, default:"odom"\)

* 里程计坐标系。

~base\_frame\_id\(string, default:"base\_link"\)

* 机器人极坐标系。

~global\_frame\_id\(string, default:"map"\)

* 地图坐标系。

.... 参数较多，上述是几个较为常用的参数，其他参数介绍可参考官网。

##### 3.6坐标变换

里程计本身也是可以协助机器人定位的，不过里程计存在累计误差且一些特殊情况时\(车轮打滑\)会出现定位错误的情况，amcl 则可以通过估算机器人在地图坐标系下的姿态，再结合里程计提高定位准确度。具体实现如下:

* 里程计定位:只是通过里程计数据实现 /odom\_frame 与 /base\_frame 之间的坐标变换。
* amcl定位: 可以提供 /map\_frame 、/odom\_frame 与 /base\_frame 之间的坐标变换。

![](/assets/amcl定位坐标变换.png)

#### 3.amcl使用

##### 3.1编写amcl节点相关的launch文件

关于launch文件的实现，在amcl功能包下的example目录已经给出了示例，可以作为参考，具体实现:

```
roscd amcl
ls examples
```

该目录下会列出两个文件: amcl\_diff.launch 和 amcl\_omni.launch 文件，前者适用于差分移动机器人，后者适用于全向移动机器人，可以按需选择，此处参考前者，新建 launch 文件，复制 amcl\_diff.launch 文件内容并修改如下:

```xml
<launch>
<node pkg="amcl" type="amcl" name="amcl" output="screen">
  <!-- Publish scans from best pose at a max of 10 Hz -->
  <param name="odom_model_type" value="diff"/><!-- 里程计模式为差分 -->
  <param name="odom_alpha5" value="0.1"/>
  <param name="transform_tolerance" value="0.2" />
  <param name="gui_publish_rate" value="10.0"/>
  <param name="laser_max_beams" value="30"/>
  <param name="min_particles" value="500"/>
  <param name="max_particles" value="5000"/>
  <param name="kld_err" value="0.05"/>
  <param name="kld_z" value="0.99"/>
  <param name="odom_alpha1" value="0.2"/>
  <param name="odom_alpha2" value="0.2"/>
  <!-- translation std dev, m -->
  <param name="odom_alpha3" value="0.8"/>
  <param name="odom_alpha4" value="0.2"/>
  <param name="laser_z_hit" value="0.5"/>
  <param name="laser_z_short" value="0.05"/>
  <param name="laser_z_max" value="0.05"/>
  <param name="laser_z_rand" value="0.5"/>
  <param name="laser_sigma_hit" value="0.2"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_model_type" value="likelihood_field"/>
  <!-- <param name="laser_model_type" value="beam"/> -->
  <param name="laser_likelihood_max_dist" value="2.0"/>
  <param name="update_min_d" value="0.2"/>
  <param name="update_min_a" value="0.5"/>

  <param name="odom_frame_id" value="odom"/><!-- 里程计坐标系 -->
  <param name="base_frame_id" value="base_footprint"/><!-- 添加机器人基坐标系 -->
  <param name="global_frame_id" value="map"/><!-- 添加地图坐标系 -->

  <param name="resample_interval" value="1"/>
  <param name="transform_tolerance" value="0.1"/>
  <param name="recovery_alpha_slow" value="0.0"/>
  <param name="recovery_alpha_fast" value="0.0"/>
</node>
</launch>
```

##### 3.2编写测试launch文件

amcl节点是不可以单独运行的，运行 amcl 节点之前，需要先加载全局地图，发布 /odom 与 /map 的静态坐标关系，还需要启动 rviz 显示定位结果，上述节点可以集成进launch文件，内容示例如下:

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find mycar_nav)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find mycar_nav)/launch/amcl.launch" />

    <!-- 对于虚拟定位，需要设置一个/odom与/map之间的静态坐标变换 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom" />

    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz"/>
</launch>
```

当然，launch文件中地图服务节点和amcl节点中的包名、文件名需要根据自己的设置修改。

##### 3.3执行

1.先启动 Gazebo 仿真环境\(此过程略\)；

2.启动键盘控制节点：

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

3.启动上一步中集成地图服务、amcl 与 rviz 的 launch 文件；

4.在启动的 rviz 中，添加RobotModel、Map组件，分别显示机器人模型与地图，添加 posearray 插件，设置topic为particlecloud来显示 amcl 预估的当前机器人的位姿，箭头越是密集，说明当前机器人处于此位置的概率越高；

5.通过键盘控制机器人运动，会发现 posearray 也随之而改变。![](/assets/amcl测试.gif)

