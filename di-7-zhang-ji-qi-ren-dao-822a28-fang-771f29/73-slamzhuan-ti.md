## 7.3 深度图像转激光数据

本节介绍ROS中的一个功能包:depthimage\_to\_laserscan，顾名思义，该功能包可以将深度图像信息转换成激光雷达信息，应用场景如下:

> 在诸多SLAM算法中，一般都需要订阅激光雷达数据用于构建地图，因为激光雷达可以感知周围环境的深度信息，而深度相机也具备感知深度信息的功能，且最初激光雷达价格比价比较昂贵，那么在传感器选型上可以选用深度相机代替激光雷达吗？

答案是可以的，不过二者发布的消息类型是完全不同的，如果想要实现传感器的置换，那么就需要将深度相机发布的三维的图形信息转换成二维的激光雷达信息，这一功能就是通过depthimage\_to\_laserscan来实现的。

#### 1.depthimage\_to\_laserscan简介

##### 1.1原理

depthimage\_to\_laserscan将实现深度图像与雷达数据转换的原理比较简单，雷达数据是二维的、平面的，深度图像是三维的，是若干二维\(水平\)数据的纵向叠加，如果将三维的数据转换成二维数据，只需要取深度图的某一层即可，为了方面理解，请看官方示例:

图一:深度相机与外部环境\(实物图\)

![](/assets/i2l_G1.jpg)

图二:深度相机发布的图片信息，图中彩线对应的是要转换成雷达信息的数据

![](/assets/i2l_G2.png)

图三:将图二以点云的方式显示更为直观，图中彩线对应的仍然是要转换成雷达信息的数据

![](/assets/i2l_G3.png)

图四:转换之后的结果图\(俯视\)

![](/assets/i2l_G4.png)

##### 1.2优缺点

**优点:**深度相机的成本一般低于激光雷达，可以降低硬件成本；

**缺点: **深度相机较之于激光雷达无论是检测范围还是精度都有不小的差距，SLAM效果可能不如激光雷达理想。

##### 1.3安装

使用之前请先安装,命令如下:

```
sudo apt-get install ros-melodic-depthimage-to-laserscan
```

#### 2.depthimage\_to\_laserscan节点说明

depthimage\_to\_laserscan 功能包的核心节点是:depthimage\_to\_laserscan ，为了方便调用，需要先了解该节点订阅的话题、发布的话题以及相关参数。

##### 2.1订阅的Topic

image\(sensor\_msgs/Image\)

* 输入图像信息。

camera\_info\(sensor\_msgs/CameraInfo\)

* 关联图像的相机信息。通常不需要重新映射，因为camera\_info将从与image相同的命名空间中进行订阅。

##### 2.2发布的Topic

scan\(sensor\_msgs/LaserScan\)

* 发布转换成的激光雷达类型数据。

##### 2.3参数

该节点参数较少，只有如下几个，一般需要设置的是: output\_frame\_id。

~scan\_height\(int, default: 1 pixel\)

* 设置用于生成激光雷达信息的象素行数。

~scan\_time\(double, default: 1/30.0Hz \(0.033s\)\)

* 两次扫描的时间间隔。

~range\_min\(double, default: 0.45m\)

* 返回的最小范围。结合range\_max使用，只会获取 range\_min 与 range\_max 之间的数据。

~range\_max\(double, default: 10.0m\)

* 返回的最大范围。结合range\_min使用，只会获取 range\_min 与 range\_max 之间的数据。

~output\_frame\_id\(str, default: camera\_depth\_frame\)

* 激光信息的ID。

#### 3.depthimage\_to\_laserscan使用

##### 3.1编写launch文件

编写launch文件执行，将深度信息转换成雷达信息

```xml
<launch>
    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan">
        <remap from="image" to="/camera/depth/image_raw" />
        <param name="output_frame_id" value="camera"  />
    </node>
</launch>
```

订阅的话题需要根据深度相机发布的话题设置，output\_frame\_id需要与深度相机的坐标系一致。

##### 3.2修改URDF文件

经过信息转换之后，深度相机也将发布雷达数据，为了不产生混淆，可以注释掉 xacro 文件中的关于激光雷达的部分内容。

##### 3.3执行

1.启动gazebo仿真环境，如下:

![](/assets/i2l_仿真.PNG)

2.启动rviz并添加相关组件\(image、LaserScan\)，结果如下:

#### ![](/assets/i2l_rviz.PNG)4.SLAM应用

现在我们已经实现并测试通过深度图像信息转换成激光雷达信息了，接下来是实践阶段，通过深度相机实现SLAM，流程如下:

1.先启动 Gazebo 仿真环境；

2.启动转换节点；

3.再启动地图绘制的 launch 文件；

4.启动键盘键盘控制节点，用于控制机器人运动建图；

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

5.在 rviz 中添加组件，显示栅格地图最后，就可以通过键盘控制gazebo中的机器人运动，同时，在rviz中可以显示gmapping发布的栅格地图数据了，但是，前面也介绍了，由于精度和检测范围的原因，尤其再加之环境的特征点偏少，建图效果可能并不理想，建图中甚至会出现地图偏移的情况。

