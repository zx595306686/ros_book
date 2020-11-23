### 7.2.5 导航与SLAM建图

> 场景:在 7.2.1 导航实现01\_SLAM建图中，我们是通过键盘控制机器人移动实现建图的，而后续又介绍了机器人的自主移动实现，那么可不可以将二者结合，实现机器人自主移动的SLAM建图呢？

上述需求是可行的。虽然可能会有疑问，导航时需要地图信息，之前导航实现时，是通过 map\_server 包的 map\_server 节点来发布地图信息的，如果不先通过SLAM建图，那么如何发布地图信息呢？SLAM建图过程中本身就会时时发布地图信息，所以无需再使用map\_server，SLAM本身就已经发布了话题为 /map 的消息了。

该过程实现比较简单，步骤如下:

1. 编写launch文件，集成SLAM、amcl与move\_base相关节点；
2. 执行launch文件并测试。

#### 1.编写launc文件

当前launch文件实现，无需调用map\_server的相关节点，只需要启动SLAM节点、amcl节点与move\_base节点，示例内容如下:

```xml
<launch>
    <!-- 启动SLAM节点 -->
    <include file="$(find mycar_nav)/launch/slam.launch" />
    <!-- 启动AMCL节点 -->
    <include file="$(find mycar_nav)/launch/amcl.launch" />

    <!-- 运行move_base节点 -->
    <include file="$(find mycar_nav)/launch/path.launch" />

    <!-- 对于虚拟定位，需要设置一个/odom与/map之间的静态坐标变换 -->
    <node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom 100" />

    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_nav)/rviz/nav.rviz" />
</launch>
```

#### 2.测试

1.首先运行gazebo仿真环境；

2.然后执行launch文件；

3.在rviz中通过2D Nav Goal设置目标点，机器人开始自主移动并建图了；

4.最后可以使用 map\_server 保存地图。

![](/assets/自主移动SLAM.gif)

