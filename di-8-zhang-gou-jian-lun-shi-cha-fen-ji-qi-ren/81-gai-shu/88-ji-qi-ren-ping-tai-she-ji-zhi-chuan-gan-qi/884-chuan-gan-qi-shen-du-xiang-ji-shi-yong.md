### 8.8.4 传感器\_深度相机使用

我们的机器人平台使用的是奥比中光Astra Pro，是一款性价比较高的深度相机。

![](/assets/奥比中光深度相机.jpg)

使用流程如下:

#### 1.硬件准备

当前直接将树莓派与深度相机使用数据线连接即可，注意:如果连接虚拟机，需要添加两个USB设备，并设置为USB3.0连接。

![](/assets/VBox添加深度相机.PNG)

#### 2.软件准备

##### 2.1安装依赖

调用如下命令:

```
sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
```

其中将$ROS\_DISTRO替换成ROS版本。

##### 2.2功能包安装

进入工作空间的src目录，下载官方功能包，命令如下:

```
git clone https://github.com/orbbec/ros_astra_camera
```

下载过程耗时，需要等待。

##### 2.3创建 astra udev rule 规则

进入astra\_camera功能包，执行内置脚本文件，命令如下:

```
roscd astra_camera
./scripts/create_udev_rules
```

并编译。

#### 3.测试

##### 3.1启动

启动深度相机相关节点，命令如下:

```
roslaunch astra_camera astra.launch
```

##### 3.2彩色图像显示

此时，可能只有深度图像，没有彩色图像，请继续安装功能包:

```
sudo apt-get install ros-melodic-uvc-camera
```

在 astra.launch 中添加内容：

```xml
<node pkg="uvc_camera" type="uvc_camera_node" name="uvc_camera" output="screen">
        <remap from="/image_raw" to="/camera/rgb/image_raw" />
</node>
<param name="width" type="int" value="320" />
<param name="height" type="int" value="240" />
<param name="fps" type="int" value="30" />
<param name="frame" type="string" value="wide_stereo" />

<param name="auto_focus" type="bool" value="False" />
<param name="focus_absolute" type="int" value="0" />
<!-- other supported params: auto_exposure, exposure_absolute, brightness, power_line_frequency -->

<param name="device" type="string" value="/dev/video1" />
<param name="camera_info_url" type="string" value="file://$(find uvc_camera)/example.yaml" />
```

##### 3.3rviz显示

启动 rviz，添加相关插件:添加PointCloud2 和 两个image，topic 分别设置为:/camera/depth/points 和 /camera/depth/image\_raw、/camera/rgb/image\_raw。

![](/assets/深度数据.PNG)

