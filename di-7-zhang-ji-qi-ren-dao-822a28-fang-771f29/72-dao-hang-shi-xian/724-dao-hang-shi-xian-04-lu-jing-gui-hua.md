### 7.2.4 导航实现04\_路径规划

毋庸置疑的，路径规划是导航中的核心功能之一，在ROS的导航功能包集navigation中提供了 move\_base 功能包，用于实现此功能。

#### 1.move\_base简介

move\_base 功能包提供了基于动作\(action\)的路径规划实现，move\_base 可以根据给定的目标点，控制机器人底盘运动至目标位置，并且在运动过程中会连续反馈机器人自身的姿态与目标点的状态信息。如前所述\(7.1\)move\_base主要由全局路径规划与本地路径规划组成。

> move\_base安装\(move\_base已经被集成到了navigation包\):
>
> `sudo apt-get install ros-<ROS版本>-navigation`

#### 2.move\_base节点说明

move\_base功能包中的核心节点是:move\_base。为了方便调用，需要先了解该节点action、订阅的话题、发布的话题、服务以及相关参数。

##### 2.1动作

**动作订阅**

move\_base/goal\(move\_base\_msgs/MoveBaseActionGoal\)

* move\_base 的运动规划目标。

move\_base/cancel\(actionlib\_msgs/GoalID\)

* 取消目标。

**动作发布**

move\_base/feedback\(move\_base\_msgs/MoveBaseActionFeedback\)

* 连续反馈的信息，包含机器人底盘坐标。

move\_base/status\(actionlib\_msgs/GoalStatusArray\)

* 发送到move\_base的目标状态信息。

move\_base/result\(move\_base\_msgs/MoveBaseActionResult\)

* 操作结果\(此处为空\)。

##### 2.2订阅的Topic

move\_base\_simple/goal\(geometry\_msgs/PoseStamped\)

* 运动规划目标\(与action相比，没有连续反馈，无法追踪机器人执行状态\)。

##### 2.3发布的Topic

cmd\_vel\(geometry\_msgs/Twist\)

* 输出到机器人底盘的运动目录。

##### 2.4服务

~make\_plan\(nav\_msgs/GetPlan\)

* 请求该服务，可以获取给定目标的规划路径，但是并不执行该路径规划。

~clear\_unknown\_space\(std\_srvs/Empty\)

* 允许用户直接清除机器人周围的未知空间。

~clear\_costmaps\(std\_srvs/Empty\)

* 允许清除代价地图中的障碍物，可能会导致机器人与障碍物碰撞，请慎用。

##### 2.5参数

暂不应用...如有需要，请参考官网介绍。

#### 3.move\_base与代价地图

##### 3.1概念

机器人导航\(尤其是路径规划模块\)是依赖于地图的，地图在SLAM时已经有所介绍了，ROS中的地图其实就是一张图片，这张图片有宽度、高度、分辨率等元数据，在图片中使用灰度值来表示障碍物存在的概率。不过SLAM构建的地图在导航中是不可以直接使用的，因为：

1. SLAM构建的地图是静态地图，而导航过程中，障碍物信息是可变的，可能障碍物被移走了，也可能添加了新的障碍物，导航中需要时时的获取障碍物信息；
2. 在靠近障碍物边缘时，虽然此处是空闲区域，但是机器人在进入该区域后可能由于其他一些因素，比如：惯性、或者不规则形体的机器人转弯时可能会与障碍物产生碰撞，安全起见，最好在地图的障碍物边缘设置警戒区，尽量禁止机器人进入...

所谓代价地图，就是在静态地图基础之上添加一些辅助信息的地图，比如时时获取的障碍物数据，基于静态地图添加的膨胀区等数据。

##### 3.2组成

代价地图有两张:global\_costmap\(全局代价地图\) 和 local\_costmap\(本地代价地图\)，前者用于全局路径规划，后者用于本地路径规划。

两张代价地图都可以多层叠加,一般有以下层级:

* Static Map Layer：静态地图层，SLAM构建的静态地图。

* Obstacle Map Layer：障碍地图层，传感器感知的障碍物信息。

* Inflation Layer：膨胀层，在以上两层地图上进行膨胀（向外扩张），以避免机器人的外壳会撞上障碍物。

* Other Layers：自定义costmap。

多个layer可以按需自由搭配。

![](/assets/导航静态效果.PNG)

##### 3.3碰撞算法

在代价地图中，如果设置地图的附加信息，那么机器人与障碍物的间距计算是重要实现，在ROS中，如何计算安全坚决呢？请看下图:

![](/assets/碰撞算法.jpg)

上图中，横轴是距离机器人中心的距离，纵轴是代价地图中栅格的灰度值。

* 致命障碍:栅格值为254，此时障碍物与机器人中心重叠，必然发生碰撞；
* 内切障碍:栅格值为253，此时障碍物处于机器人的内切圆内，必然发生碰撞；
* 外切障碍:栅格值为\[128,252\]，此时障碍物处于其机器人的外切圆内，处于碰撞临界，不一定发生碰撞；
* 非自由空间:栅格值为\(0,127\]，此时机器人处于障碍物附近，属于危险警戒区，进入此区域，将来可能会发生碰撞；
* 自由区域:栅格值为0，此处机器人可以自由通过；
* 未知区域:栅格值为255，还没探明是否有障碍物。

膨胀空间的设置可以参考非自由空间。

#### 3.move\_base使用

路径规划算法在move\_base功能包的move\_base节点中已经封装完毕了，但是还不可以直接调用，因为算法虽然已经封装了，但是该功能包面向的是各种类型支持ROS的机器人，不同类型机器人可能大小尺寸不同，传感器不同，速度不同，应用场景不同....最后可能会导致不同的路径规划结果，那么在调用路径规划节点之前，我们还需要配置机器人参数。具体实现如下:

1. 先编写launch文件模板
2. 编写配置文件
3. 集成导航相关的launch文件
4. 测试

##### 3.1launch文件

关于move\_base节点的调用，模板如下:

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

launch文件解释:

启动了 move\_base 功能包下的 move\_base 节点，respawn 为 false，意味着该节点关闭后，不会被重启；clear\_params 为 true，意味着每次启动该节点都要清空私有参数然后重新载入；通过 rosparam 会载入若干 yaml 文件用于配置参数，这些yaml文件的配置以及作用详见下一小节内容。

##### 3.2配置文件

关于配置文件的编写，可以参考一些成熟的机器人的路径规划实现，比如: turtlebot3，github链接：[https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3\_navigation/param](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_navigation/param)，先下载这些配置文件备用。

在功能包下新建 param 目录，复制下载的文件到此目录: costmap\_common\_params\_burger.yaml、local\_costmap\_params.yaml、global\_costmap\_params.yaml、base\_local\_planner\_params.yaml，并将costmap\_common\_params\_burger.yaml 重命名为:costmap\_common\_params.yaml。

配置文件修改以及解释:

###### 3.2.1costmap\_common\_params.yaml

该文件是move\_base 在全局路径规划与本地路径规划时调用的通用参数，包括:机器人的尺寸、距离障碍物的安全距离、传感器信息等。配置参考如下:

```yaml
#机器人几何参，如果机器人是圆形，设置 robot_radius,如果是其他形状设置 footprint
robot_radius: 0.12 #圆形
# footprint: [[-0.12, -0.12], [-0.12, 0.12], [0.12, 0.12], [0.12, -0.12]] #其他形状

obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到距离小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代码地图中 3.5 米以外的障碍物


#膨胀半径，扩展在碰撞区域以外的代价区域，使得机器人规划路径避开障碍物
inflation_radius: 0.2
#代价比例系数，越大则代价值越小
cost_scaling_factor: 3.0

#地图类型
map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，你可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}
```

###### 

###### 3.2.2global\_costmap\_params.yaml

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

###### 3.2.3local\_costmap\_params.yaml

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

###### 3.2.4base\_local\_planner\_params

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

###### 3.2.5参数配置技巧

以上配置在实操中，可能会出现机器人在本地路径规划时与全局路径规划不符而进入膨胀区域出现假死的情况，如何尽量避免这种情形呢？

> 全局路径规划与本地路径规划虽然设置的参数是一样的，但是二者路径规划和避障的职能不同，可以采用不同的参数设置策略:
>
> * 全局代价地图可以将膨胀半径和障碍物系数设置的偏大一些；
> * 本地代价地图可以将膨胀半径和障碍物系数设置的偏小一些。
>
> 这样，在全局路径规划时，规划的路径会尽量远离障碍物，而本地路径规划时，机器人即便偏离全局路径也会和障碍物之间保留更大的自由空间，从而避免了陷入“假死”的情形。

##### 3.3launch文件集成

如果要实现导航，需要集成地图服务、amcl 、move\_base 与 Rviz 等，集成示例如下:

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

##### 3.4测试

1.先启动 Gazebo 仿真环境\(此过程略\)；

2.启动导航相关的 launch 文件；

3.添加Rviz组件\(参考演示结果\),可以将配置数据保存，后期直接调用；

全局代价地图与本地代价地图组件配置如下:

![](/assets/rviz代价地图.PNG)

全局路径规划与本地路径规划组件配置如下:

![](/assets/rviz路径规划.PNG)

4.通过Rviz工具栏的 2D Nav Goal设置目的地实现导航。

![](/assets/导航.gif)5.也可以在导航过程中，添加新的障碍物，机器人也可以自动躲避障碍物。

