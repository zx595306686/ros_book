## 9.2 导航实现

本节介绍实体机器人导航的基本实现流程。该流程实现与7.2节内容类似，主要内容仍然集中在SLAM、地图服务、定位与路径规划，本节内容不再重复介绍7.2节中各个知识点的实现细节，而是注重知识点应用。

实体机器人导航实现流程如下:

1. 准备工作
2. SLAM实现
3. 地图保存
4. 定位实现
5. 路径规划
6. 导航集成
7. 测试

#### 1.准备工作

##### 1.0分布式架构

分布式架构搭建完毕且能正常运行，在PC端远程登陆机器人端。

##### 1.1PC端

**准备机器人模型文件\(可选\)**

在实体机器人导航中，可以选择是否在rviz显示机器人模型，如果选用机器人模型，那么在机器人URDF文件中需要去除gazebo仿真插件相关实现，可以参考 6.4.4节内容。

新建导航实现功能包，导入依赖: gmapping map\_server amcl move\_base。

##### 1.2机器人端

机器人端需要准备底盘以及传感器启动的launch文件，launch文件有二。

1.使用机器人模型时所需的的launch文件:

```xml
<!-- 机器人启动文件：
        1.启动底盘
        2.启动激光雷达
        3.启动摄像头
 -->

<launch>
    <include file="$(find ros_arduino_python)/launch/arduino.launch" />
    <include file="$(find rplidar_ros)/launch/rplidar.launch" />
    <include file="$(find astra_camera)/launch/astra.launch" />
</launch>
```

PS:当使用机器人模型时，需要注意，模型中的传感器坐标系必须与实体传感器坐标系一致，且需要注意雷达偏航角度的设置。

2.不使用机器人模型时所需的launch文件:

```xml
<launch>
    <include file="$(find 步骤1的功能包)/launch/步骤1的launch文件" />
    <!-- rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏行角度、y俯仰角度、x翻过角度 父级坐标系 子级坐标系 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="laser2basefootprint" args="0 0 0.16 3.14159 0 0  /base_footprint /laser" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera2basefootprint" args="0.08 0 0.13 0 0 0  /base_footprint /camera_link" />
</launch>
```

##### 1.3结果演示

不使用机器人模型时，启动机器人底盘以及rviz，rviz中结果\(此时显示机器人模型异常，且TF中只有代码中发布的坐标变换\):

![](/assets/实体机器人导航%28不使用机器人模型%29rviz.PNG)

不使用机器人模型时，启动机器人底盘以及rviz，rviz中结果\(此时显示机器人模型，且TF坐标变换正常\):

![](/assets/实体机器人导航%28使用机器人模型%29rviz.PNG)

后续，在导航时使用机器人模型。

#### 2.SLAM实现

SLAM实现优先以gmapping为例，编写launch文件如下\(与仿真一致\):

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

#### 3.地图保存

地图保存launch文件示例如下\(与仿真一致\):

```xml
<launch>
    <arg name="filename" value="$(find mycar_nav)/map/nav" />
    <node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```

#### 4.定位实现

amcl相关launch文件示例如下\(与仿真一致\):

```xml
<launch>
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <!-- Publish scans from best pose at a max of 10 Hz -->

        <param name="min_particles" value="500"/> 
        <param name="max_particles" value="5000"/> 
        <param name="kld_err" value="0.05"/> 
        <param name="kld_z" value="0.99"/> 
        <param name="update_min_d" value="0.2"/> 
        <param name="update_min_a" value="0.5"/> 
        <param name="resample_interval" value="1"/> 
        <param name="transform_tolerance" value="0.1"/> 
        <param name="recovery_alpha_slow" value="0.0"/>
        <param name="recovery_alpha_fast" value="0.0"/> 
        <param name="gui_publish_rate" value="10.0"/> 
        <param name="save_pose_rate" value="0.5"/> 
        <param name="use_map_topic" value="false"/> 
        <param name="first_map_only" value="false"/> 

        <param name="laser_min_range" value="-1.0"/> 
        <param name="laser_max_range" value="-1.0"/> 
        <param name="laser_max_beams" value="30"/> 
        <param name="laser_z_hit" value="0.5"/> 
        <param name="laser_z_short" value="0.05"/> 
        <param name="laser_z_max" value="0.05"/>
        <param name="laser_z_rand" value="0.5"/> 
        <param name="laser_sigma_hit" value="0.2"/>
        <param name="laser_lambda_short" value="0.1"/> 
        <param name="laser_likehood_max_dist" value="2.0"/> 
        <param name="laser_model_type" value="likelihood_field"/> 

        <param name="odom_model_type" value="diff"/> 
        <param name="odom_alpha1" value="0.2"/>
        <param name="odom_alpha2" value="0.2"/> 
        <!-- translation std dev, m -->
        <param name="odom_alpha3" value="0.8"/>
        <param name="odom_alpha4" value="0.2"/>
        <param name="odom_alpha5" value="0.1"/> 
        <param name="odom_frame_id" value="odom"/>
        <param name="base_frame_id" value="base_footprint"/> 
        <param name="global_frame_id" value="map"/> 
        <param name="tf_broadcast" value="true"/> 

        <param name="initial_pose_x" value="0.0"/>
        <param name="initial_pose_y" value="0.0"/> 
        <param name="initial_pose_a" value="0.0"/> 
        <param name="initial_cov_xx" value="0.5*0.5"/> 
        <param name="initial_cov_yy" value="0.5*0.5"/> 
        <param name="initial_cov_aa" value="(π/12)*(π/12)"/> 
    </node>
</launch>
```

#### 5.路径规划

move\_base相关示例文件如下\(与仿真一致\):

```xml
<launch>

    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
        <rosparam file="$(find 功能包)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find 功能包)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find 功能包)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find 功能包)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find 功能包)/param/base_local_planner_params.yaml" command="load" />
    </node>

</launch>
```

相关配置文件如下:

###### costmap\_common\_params.yaml

该文件是move\_base 在全局路径规划与本地路径规划时调用的通用参数，包括:机器人的尺寸、距离障碍物的安全距离、传感器信息等。配置参考如下:

```yaml
#机器人几何参数
robot_radius: 0.12 #圆形
# footprint: [[-0.25, -0.25], [-0.25, 0.25], [0.25, 0.25], [0.25, -0.25]] #其他

obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到距离 小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代码地图中 3.5 米以外的障碍物


#机器人和障碍物之间需要保持的最小距离
#inflation_radius: 0.2
#cost_scaling_factor: 10.0

map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，你可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}
```

###### 

###### global\_costmap\_params.yaml

该文件用于全局代价地图参数设置:

```yaml
global_costmap:
  global_frame: map #地图坐标系
  robot_base_frame: base_footprint #机器人坐标系
  # 以此实现坐标变换

  update_frequency: 1.0 #代价地图更新频率
  publish_frequency: 1.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: true # 是否使用一个地图或者地图服务器来初始化全局代价地图，如果不使用静态地图，这个参数为false.
```

###### local\_costmap\_params.yaml

该文件用于局部代价地图参数设置:

```yaml
local_costmap:
  global_frame: odom #里程计坐标系
  robot_base_frame: base_footprint #机器人坐标系

  update_frequency: 10.0 #代价地图更新频率
  publish_frequency: 10.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: false  #不需要静态地图，可以提升导航效果
  rolling_window: true #是否使用动态窗口，默认为false，在静态的全局地图中，地图不会变化
  width: 3 # 局部地图宽度 单位是 m
  height: 3 # 局部地图高度 单位是 m
  resolution: 0.05 # 局部地图分辨率 单位是 m，一般与静态地图分辨率保持一致
```

###### base\_local\_planner\_params

基本的局部规划器参数配置，这个配置文件设定了机器人的最大和最小速度限制值，也设定了加速度的阈值。

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

#### 6.导航集成

导航相关 launch 文件示例如下:

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find mycar_nav)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find mycar_nav)/launch/amcl.launch" />

    <!-- 运行move_base节点 -->
    <include file="$(find mycar_nav)/launch/path.launch" />

    <!-- 对于虚拟定位，需要设置一个/odom与/map之间的静态坐标变换 -->
    <!-- <node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom 100" /> -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom" />
    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_nav)/rviz/nav.rviz" />

</launch>
```

另外，根据之前介绍，还可以实现自动SLAM，相关launch文件示例如下:

```xml
<launch>
    <!-- 启动SLAM节点 -->
    <include file="$(find mycar_nav)/launch/slam.launch" />
    <!-- 启动AMCL节点 -->
    <include file="$(find mycar_nav)/launch/amcl.launch" />

    <!-- 运行move_base节点 -->
    <include file="$(find mycar_nav)/launch/path.launch" />

    <!-- 对于虚拟定位，需要设置一个/odom与/map之间的静态坐标变换 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom" />
    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_nav)/rviz/nav.rviz" />
</launch>
```

#### 7.测试

1.PC端，启动 roscore；

2.机器人端，启动机器人底盘；

3.PC端，启动机器人模型加载节点；

4.PC端，启动SLAM建图与键盘控制节点，控制机器人运动，并建图；或者也可以启动自动SLAM建图，通过rviz控制机器人；

5.PC端，建图完毕后，启动map\_server的地图保存节点；

6.PC端，启动导航节点，通过 rviz 实现导航。

参考视频: 附件资料自动SLAM建图。

