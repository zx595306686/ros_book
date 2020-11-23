### 7.4.1 SLAM实现之hector\_slm

之前介绍的gmapping是高度依赖于里程计的，对里程计的高度依赖导致在一些场景下gmapping可能无法实现SLAM，比如:

> 手持设备、无人机、地面不平坦、水面机器人....

在这些场景中，可能不能提供准确的里程计信息，甚至不能提供里程计信息，此时gmapping就不是最好的选择了，其中一种替代方案就是: hector\_slam。

#### 1.hector\_slam简介

hector\_slam 是一个元功能包，在 hector\_slam 中包含了核心功能包hector\_mapping以及与hector\_mapping相关的其他功能包，hector\_mapping是一种SLAM算法实现，它使用高斯牛顿方法，无需里程计数据，可以根据激光雷达信息估算里程计数据从而实现SLAM，所以较之于 gmapping，hector\_mapping更适用于不能提供或不能准确提供里程计信息的场景，简化了SLAM的实现环节，但是对雷达传感器性能要求较高\(建议雷达的更新频率高于40hz\)。

> 安装命令:
>
> ```
> sudo apt install ros-melodic-hector-slam
> ```

#### 2.hector\_slam节点说明

hector\_slam 功能包中的核心功能包是:hector\_mapping，hector\_mapping的核心节点是:hector\_mapping为了方便调用，需要先了解该节点订阅的话题、发布的话题、服务以及相关参数。

##### 2.1订阅的Topic

scan\(sensor\_msgs/LaserScan\)

* SLAM所需的雷达信息。

syscommand\(std\_msgs/String\)

* 系统命令。如果字符串等于“重置”，则将地图和机器人姿态重置为其初始状态。

##### 2.2发布的Topic

map\_metadata\(nav\_msgs/MapMetaData\)

* 地图元数据，包括地图的宽度、高度、分辨率等，该消息会固定更新。

map\(nav\_msgs/OccupancyGrid\)

* 地图栅格数据，一般会在rviz中以图形化的方式显示。

slam\_out\_pose\(geometry\_msgs/PoseStamped\)

* 估计的机器人姿态\(无协方差\)。

poseupdate\(geometry\_msgs/PoseWithCovarianceStamped\)

* 估计的机器人姿态\(具有高斯估计的不确定性\)。

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

~pub\_map\_odom\_transform\(bool, default: true\)

* 是否发布 map 于 odom 之间的坐标变换。

.... 参数较多，上述是几个较为常用的参数，其他参数介绍可参考官网。

##### 2.5所需的坐标变换

雷达坐标系→基坐标系

* 一般由 robot\_state\_publisher 或 static\_transform\_publisher 发布。

##### 2.6发布的坐标变换

地图坐标系→里程计坐标系

* 地图中里程计的估算值\(仅在在参数pub\_map\_odom\_transform为true时发布\)。

#### 3.hector\_slam使用

##### 3.1编写hector\_mapping节点相关的launch文件

编写 hector\_slam 节点，可以参考: hector\_mappin 包下的  mapping\_default.launch 文件，复制文件内容并修改如下:

```xml
<launch>
  <arg name="tf_map_scanmatch_transform_frame_name" default="scanmatcher_frame"/>
  <arg name="base_frame" default="base_footprint"/>
  <arg name="odom_frame" default="odom"/>
  <arg name="pub_map_odom_transform" default="true"/>
  <arg name="scan_subscriber_queue_size" default="5"/>
  <arg name="scan_topic" default="scan"/>
  <arg name="map_size" default="2048"/>

  <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">

    <!-- 重点 -->
    <param name="map_frame" value="map" />
    <param name="base_frame" value="$(arg base_frame)" />
    <param name="odom_frame" value="$(arg odom_frame)" />

    <!-- Tf use -->
    <param name="use_tf_scan_transformation" value="true"/>
    <param name="use_tf_pose_start_estimate" value="false"/>
    <param name="pub_map_odom_transform" value="$(arg pub_map_odom_transform)"/>

    <!-- Map size / start point -->
    <param name="map_resolution" value="0.050"/>
    <param name="map_size" value="$(arg map_size)"/>
    <param name="map_start_x" value="0.5"/>
    <param name="map_start_y" value="0.5" />
    <param name="map_multi_res_levels" value="2" />

    <!-- Map update parameters -->
    <param name="update_factor_free" value="0.4"/>
    <param name="update_factor_occupied" value="0.9" />    
    <param name="map_update_distance_thresh" value="0.4"/>
    <param name="map_update_angle_thresh" value="0.06" />
    <param name="laser_z_min_value" value = "-1.0" />
    <param name="laser_z_max_value" value = "1.0" />

    <!-- Advertising config --> 
    <param name="advertise_map_service" value="true"/>

    <param name="scan_subscriber_queue_size" value="$(arg scan_subscriber_queue_size)"/>
    <!-- 重点 -->
    <param name="scan_topic" value="$(arg scan_topic)"/>

    <!-- Debug parameters -->
    <!--
      <param name="output_timing" value="false"/>
      <param name="pub_drawings" value="true"/>
      <param name="pub_debug_output" value="true"/>
    -->
    <param name="tf_map_scanmatch_transform_frame_name" value="$(arg tf_map_scanmatch_transform_frame_name)" />
  </node>

  <!--<node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 map nav 100"/>-->
</launch>
```

launch文件中的map\_frame、base\_frame、scan\_topic、odom\_frame、pub\_map\_odom\_transform是重点需要修改的参数，其中前面三个参数: map\_frame、base\_frame、scan\_topic按需设置即可，后两者odom\_frame、pub\_map\_odom\_transform则需要视情况而定。

**情况1:** 如果机器人平台已经提供里程计坐标系，那么可以将pub\_map\_odom\_transform参数设置为true，odom\_frame参数设置为里程计坐标系，示例如下:

```xml
 <param name="pub_map_odom_transform" value="true"/>
 <param name="odom_frame" value="里程计坐标系" />
```

**情况2:**如果机器人平台不提供里程计坐标系，那么可以将pub\_map\_odom\_transform参数设置为true，odom\_frame参数设置为机器人基坐标系，直接发布地图到基坐标系的转换，示例如下:

```xml
 <param name="pub_map_odom_transform" value="true"/>
 <param name="odom_frame" value="机器人基坐标系" />
```

**情况3:**如果另一个节点负责发布map-&gt;odom或map-&gt;base\_frame转换，那么hector\_mappin可以不再广播地图到里程计的坐标变换，可以将pub\_map\_odom\_transform参数设置为false，odom\_frame无需设置，示例如下:

```xml
<param name="pub_map_odom_transform" value="false"/>
```

##### 3.2执行

此过程于gmapping执行流程类似。

1.先启动 Gazebo 仿真环境；

2.然后再启动地图绘制的 launch 文件；

3.启动键盘键盘控制节点，用于控制机器人运动建图

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

注意:如果机器人旋转速度过快，在建图时可能会出现地图漂移的情况，可以将键盘控制的角速度调小，命令如下:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _turn:=0.2
```

4.在 rviz 中添加组件，显示栅格地图

![](/assets/hector建图效果.PNG)

5.保存地图也是可以使用 map\_server 保存。

PS:如果机器人旋转过快，可能会出现地图漂移、叠加的情况。

![](/assets/hector_mapping漂移.PNG)

