### 8.8.5 传感器\_集成

接下来，我们就可以将传感器\(激光雷达与深度相机\)集成到机器人本体并测试了，大致流程如下:

1. 硬件组装；
2. 树莓派端程序安装以及实现；
3. 启动并测试。

#### 1.硬件组装

硬件组装效果图如下:

![](/assets/传感器集成效果图.PNG)

#### 2.树莓派端实现

在树莓派端需要启动底盘控制、深度相机与激光雷达相关节点，那么对应的也需要安装这三部分的功能包，这些功能包我们之前已经下载、修改了相关配置且调试通过了，当然，如果之前是在PC端实现的\(比如:深度相机或激光雷达相关功能包\)，直接通过ssh上传至树莓派的工作空间并重新编译即可，如果之前是在树莓派端实现的，无需做任何修改。

在此，我们主要是需要优化这些节点的启动，主要优化有两点:

1. 简化机器人启动：如果逐一启动底盘控制、深度相机与激光雷达显然操作冗余，可以直接整合进一个launch文件；
2. 发布坐标变换：如果只是简单的启动这些节点，那么在 rviz 中显示时，会发现出现了TF转换异常，比如参考坐标系设置为odom时，雷达信息显示失败；再比如TF插件中会给出警告信息这是因为没有发布坐标变换信息，因此还需要设置坐标变换。

##### 1.launch文件

新建功能包并创建launch文件，文件名自定义，内容如下:

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

##### 2.坐标变换

如果启动时加载了机器人模型，且模型中设置的坐标系名称与机器人实体中设置的坐标系一致，那么可以不再添加坐标变换，因为机器人模型可以发布坐标变换信息，如果没有启动机器人模型，就需要自定义坐标变换实现了，继续新建launch文件，内容如下:

```xml
<!-- 机器人启动文件：
        当不包含机器人模型时，需要发布坐标变换
 -->

<launch>
    <include file="$(find 步骤1的功能包)/launch/步骤1的launch文件" />
    <!-- rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏行角度、y俯仰角度、x翻过角度 父级坐标系 子级坐标系 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="laser2basefootprint" args="0 0 0.16 3.14159 0 0  /base_footprint /laser" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera2basefootprint" args="0.08 0 0.13 0 0 0  /base_footprint /camera_link" />
</launch>
```

当启动该launch文件时，就可以发布坐标变换信息了。

#### 3.测试

最后，就可以启动PC端与树莓派端相关节点并运行查看结果了:

##### 1.PC端

首先启动 roscore:

```
roscore
```

然后启动键盘控制节点:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

还需要启动rviz:

```
rviz
```

##### 2.树莓派

直接执行上一步的机器人启动launch文件:

```
roslaunch 自定义包 自定义launch文件
```

##### 3.结果显示

在rviz中添加laserscan、image等插件，并通过键盘控制机器人运动，查看rviz中的显示结果:

![](/assets/机器人硬件集成测试.PNG)

