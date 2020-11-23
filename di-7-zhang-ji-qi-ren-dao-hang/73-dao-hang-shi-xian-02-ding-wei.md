## 7.3 导航实现02\_定位

定位实现，使用的是ros的navigation包中的amcl包，实现步骤如下:

1. 安装功能包
2. 配置 amcl 的 launch 文件
3. 编写测试 amcl 的 launch 文件，需要配置静态地图
4. 执行，并查看结果

#### 1.安装功能包

当前实现定位需要的是`acml`包，该功能包已经直接集成进 navigation 包了，在此可以直接安装 navigation 包:`sudo apt-get install ros-noetic-navigation`

#### 2.编写 launch 文件

```xml
<launch>
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <!-- Publish scans from best pose at a max of 10 Hz -->

    //全部滤波器参数
    <param name="min_particles" value="500"/>   //允许的粒子数量的最小值，默认100
    <param name="max_particles" value="5000"/> //允许的例子数量的最大值，默认5000
    <param name="kld_err" value="0.05"/>    //真实分布和估计分布之间的最大误差，默认0.01
    <param name="kld_z" value="0.99"/>   //上标准分位数（1-p），其中p是估计分布上误差小于kld_err的概率，默认0.99
    <param name="update_min_d" value="0.2"/>   //在执行滤波更新前平移运动的距离，默认0.2m
    <param name="update_min_a" value="0.5"/>   //执行滤波更新前旋转的角度，默认pi/6 rad
    <param name="resample_interval" value="1"/>   //在重采样前需要的滤波更新的次数,默认2
    <param name="transform_tolerance" value="0.1"/>  //tf变换发布推迟的时间，为了说明tf变换在未来时间内是可用的
    <param name="recovery_alpha_slow" value="0.0"/> //慢速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.001是一个不错的值
    <param name="recovery_alpha_fast" value="0.0"/>  //快速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.1是个不错的值
    <param name="gui_publish_rate" value="10.0"/>  //扫描和路径发布到可视化软件的最大频率，设置参数为-1.0意为失能此功能，默认-1.0
    <param name="save_pose_rate" value="0.5"/>  //存储上一次估计的位姿和协方差到参数服务器的最大速率。被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。
    <param name="use_map_topic" value="false"/>  //当设置为true时，AMCL将会订阅map话题，而不是调用服务返回地图。也就是说，当设置为true时，有另外一个节点实时的发布map话题，也就是机器人在实时的进行地图构建，并供给amcl话题使用；当设置为false时，通过map server，也就是调用已经构建完成的地图。在navigation 1.4.2中新加入的参数。
    <param name="first_map_only" value="false"/>  //当设置为true时，AMCL将仅仅使用订阅的第一个地图，而不是每次接收到新的时更新为一个新的地图，在navigation 1.4.2中新加入的参数。

    //激光模型参数
    <param name="laser_min_range" value="-1.0"/>  //被考虑的最小扫描范围；参数设置为-1.0时，将会使用激光上报的最小扫描范围
    <param name="laser_max_range" value="-1.0"/>  //被考虑的最大扫描范围；参数设置为-1.0时，将会使用激光上报的最大扫描范围
    <param name="laser_max_beams" value="30"/>   //更新滤波器时，每次扫描中多少个等间距的光束被使用
    <param name="laser_z_hit" value="0.5"/> //模型的z_hit部分的最大权值，默认0.95
    <param name="laser_z_short" value="0.05"/> //模型的z_short部分的最大权值，默认0.1
    <param name="laser_z_max" value="0.05"/> //模型的z_max部分的最大权值，默认0.05
    <param name="laser_z_rand" value="0.5"/> //模型的z_rand部分的最大权值，默认0.05
    <param name="laser_sigma_hit" value="0.2"/> //被用在模型的z_hit部分的高斯模型的标准差，默认0.2m
    <param name="laser_lambda_short" value="0.1"/> //模型z_short部分的指数衰减参数，默认0.1
    <param name="laser_likehood_max_dist" value="2.0"/> //地图上做障碍物膨胀的最大距离，用作likehood_field模型
    <param name="laser_model_type" value="likelihood_field"/> //模型使用，可以是beam, likehood_field, likehood_field_prob（和likehood_field一样但是融合了beamskip特征），默认是“likehood_field”    

    //里程计模型参数
    <param name="odom_model_type" value="diff"/> //模型使用，可以是"diff", "omni", "diff-corrected", "omni-corrected",后面两  个是对老版本里程计模型的矫正，相应的里程计参数需要做一定的减小
    <param name="odom_alpha1" value="0.2"/> //指定由机器人运动部分的旋转分量估计的里程计旋转的期望噪声，默认0.2
    <param name="odom_alpha2" value="0.2"/> //制定由机器人运动部分的平移分量估计的里程计旋转的期望噪声，默认0.2
    <!-- translation std dev, m -->
    <param name="odom_alpha3" value="0.8"/> //指定由机器人运动部分的平移分量估计的里程计平移的期望噪声，默认0.2
    <param name="odom_alpha4" value="0.2"/> //指定由机器人运动部分的旋转分量估计的里程计平移的期望噪声，默认0.2
    <param name="odom_alpha5" value="0.1"/> //平移相关的噪声参数（仅用于模型是“omni”的情况）
    <param name="odom_frame_id" value="odom"/>  //里程计默认使用的坐标系
    <param name="base_frame_id" value="base_footprint"/>  //用作机器人的基坐标系
    <param name="global_frame_id" value="map"/>  //由定位系统发布的坐标系名称
    <param name="tf_broadcast" value="true"/>  //设置为false阻止amcl发布全局坐标系和里程计坐标系之间的tf变换

    //机器人初始化数据设置
    <param name="initial_pose_x" value="0.0"/> //初始位姿均值（x），用于初始化高斯分布滤波器。
    <param name="initial_pose_y" value="0.0"/> //初始位姿均值（y），用于初始化高斯分布滤波器。
    <param name="initial_pose_a" value="0.0"/> //初始位姿均值（yaw），用于初始化高斯分布滤波器。
    <param name="initial_cov_xx" value="0.5*0.5"/> //初始位姿协方差（x*x），用于初始化高斯分布滤波器。
    <param name="initial_cov_yy" value="0.5*0.5"/> //初始位姿协方差（y*y），用于初始化高斯分布滤波器。
    <param name="initial_cov_aa" value="(π/12)*(π/12)"/> //初始位姿协方差（yaw*yaw），用于初始化高斯分布滤波器。
  </node>
</launch>
```

#### 3.编写测试amcl的launch文件

上面的 launch 文件是无法单独运行的，启动 amcl 定位功能时，还需要结合静态地图，相关示例如下:

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find demo03_nav)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find demo03_nav)/launch/demo03_amcl.launch" />

    <!-- 对于虚拟定位，需要设置一个/odom与/map之间的静态坐标变换 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom" />

    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo03_nav)/rviz/build_map.rviz" />

</launch>
```

注意: 该文件是非必须文件，只是作为测试使用

#### 4.测试

1.先启动 Gazebo 仿真环境\(此过程略\)

2.启动键盘控制节点

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

3.启动上一步中集成 amcl 与 rviz 的 launch 文件

4.在启动的 rviz 中，添加 posearray 插件来显示 amcl 预估的当前机器人的位姿

箭头越是密集，说明当前机器人处于此位置的概率更高

5.通过键盘控制机器人运动，会发现 posearray 也随之而改变

![](/assets/15amcl定位演示.PNG)

