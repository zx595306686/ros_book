## 7.4 导航实现03\_路径规划

路径规划实现，使用的是ros的navigation包中的move\_base包，实现步骤如下:

1. 安装功能包
2. 编写 launch 文件以及move\_base所需的配置文件
3. 编写导航集成的launch文件
4. 执行，并查看结果

#### 1.安装功能包

调用 amcl 时，已经安装了 navigation 包，而路径规划相关的功能包 move\_base 也已经集成进了 navigation 包，换言之，路径规划相关功能包无需另行安装。

#### 2.编写 launch 文件以及所需的配置文件

```xml
<!-- 路径规划节点 -->
<launch>

  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
    <rosparam file="$(find 自定义包名)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find 自定义包名)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find 自定义包名)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find 自定义包名)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find 自定义包名)/param/base_local_planner_params.yaml" command="load" />
  </node>

</launch>
```

**costmap\_common\_params.yaml**

该文件为代价地图通用参数配置文件，这些参数会被用于local\_costmap和global\_costmap

```yaml
obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到举例 小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代码地图中 3.5 米以外的障碍物

#footprint：将机器人的几何参数告诉导航功能包集
#footprint: [[x0, y0], [x1, y1], ... [xn, yn]]
footprint: [[-0.25, -0.25], [-0.25, 0.25], [0.25, 0.25], [0.25, -0.25]]
#根据机器人形状以及大小设置

#机器人和障碍物之间需要保持的最小距离
inflation_radius: 0.2
cost_scaling_factor: 3.0

map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，你可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: base_scan, data_type: LaserScan, topic: scan, marking: true, clearing: true}
```

**global\_costmap\_params.yaml**

全局代价地图

```yaml
global_costmap:
  global_frame: map #地图坐标系
  robot_base_frame: base_footprint #机器人坐标系
  # 以此实现坐标变换

  update_frequency: 10.0 #地图更新频率
  publish_frequency: 10.0 #
  transform_tolerance: 0.5

  static_map: true # 是否使用一个地图或者地图服务器来初始化全局代价地图，如果不使用静态地图，这个参数为false.
```

**local\_costmap\_params.yaml**

局部代价地图

```yaml
local_costmap:
  global_frame: odom #里程计坐标系
  robot_base_frame: base_footprint #机器人坐标系

  update_frequency: 10.0 #更新频率
  publish_frequency: 10.0 #发布消息频率
  transform_tolerance: 0.5 

  static_map: false  #不需要静态地图
  rolling_window: true
  width: 1 # 局部地图宽度 单位是 m
  height: 1 # 局部地图高度 单位是 m
  resolution: 0.05 # 局部地图分辨率 单位是 m
```

**base\_local\_planner\_params.yaml**

基本的局部规划器参数配置，这个配置文件设定了机器人的最大和最小速度限制值，也设定了加速度的限值。

```yaml
TrajectoryPlannerROS:

# Robot Configuration Parameters
  max_vel_x: 0.5 # X 方向最大速度
  min_vel_x: 0.1 # X 方向最小速速

  max_vel_theta:  1.0 # 
  min_vel_theta: -1.0
  min_in_place_vel_theta: 1.0

  acc_lim_x: 1.0 # X 加速限制
  acc_lim_y: 0.0 # Y 加速限制
  acc_lim_theta: 0.6 # 角速度加速限制

# Goal Tolerance Parameters，目标公差
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05

# Differential-drive robot configuration
# 是否是全向移动机器人
  holonomic_robot: false

# Forward Simulation Parameters，前进模拟参数
  sim_time: 0.8
  vx_samples: 18
  vtheta_samples: 20
  sim_granularity: 0.05
```

#### 3.集成导航相关的 launch 文件

如果要实现导航，需要集成地图服务、amcl 、move\_base 与 Rviz 等，集成示例如下:

```xml
<!-- 导航实现 -->
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find mycar_description)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find mycar_description)/launch/mycar04_amcl.launch" />

    <!-- 运行move_base节点 -->
    <include file="$(find mycar_description)/launch/mycar05_move_base.launch" />

    <!-- 对于虚拟定位，需要设置一个/odom与/map之间的静态坐标变换 -->
    <node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom 100" />

    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_description)/rviz/nav.rviz" />

</launch>
```

#### 4.运行 launch 文件并实现导航

1.先启动 Gazebo 仿真环境\(此过程略\)

2.启动导航相关的 launch 文件

3.配置Rviz,并将配置后的文件保存，后期直接调用

4.通过Rviz工具栏的 2D Nav Goal设置目的地实现导航

![](/assets/机器人导航4.gif)

