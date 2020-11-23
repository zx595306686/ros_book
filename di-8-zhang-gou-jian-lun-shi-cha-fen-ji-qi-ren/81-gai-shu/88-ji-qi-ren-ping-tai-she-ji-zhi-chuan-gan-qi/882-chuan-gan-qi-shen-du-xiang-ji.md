### 8.8.2 传感器\_激光雷达使用

我们的机器人平台使用的是思岚A1激光雷达，这是一款成本较低的单线激光雷达。

![](/assets/思岚A1.jpg)

使用流程如下:

#### 1.硬件准备

##### 1.雷达连接上位机

当前直接连接树莓派即可，如果连接的是虚拟机，注意VirtualBox或VMware的相关设置。

![](/assets/VBox添加激光雷达.PNG)

##### 2.确认当前的 USB 转串口终端并修改权限

USB查看命令:

```
ll /dev/ttyUSB*
```

授权\(将当前用户添加进dialout组，与arduino类似\):

```
sudo usermod -a -G dialout your_user_name
```

不要忘记重启，重启之后才可以生效。

#### 2.软件准备

下载相关雷达驱动包，下载命令如下:

```
git clone https://github.com/slamtec/rplidar_ros
```

#### 3.启动并测试

##### 1.rplidar.launch文件准备

首先确认端口,编辑 rplidar.launch 文件

```xml
<launch>
  <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="serial_port"         type="string" value="/dev/ttyUSB1"/>
  <param name="serial_baudrate"     type="int"    value="115200"/><!--A1/A2 -->
  <!--param name="serial_baudrate"     type="int"    value="256000"--><!--A3 -->
  <param name="frame_id"            type="string" value="laser"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
</launch>
```

需要根据实际情况修改`<param name="serial_port" type="string" value="/dev/ttyUSB1"/>`的value值，当前是 /dev/ttyUSB1,另外 frame\_id 也可以修改，当使用URDF显示机器人模型时，需要与 URDF 中雷达 id 一致

##### 2.终端中执行 launch 文件

终端命令:`roslaunch rplidar_ros rplidar.launch`如果异常，雷达开始旋转

另启终端:`rostopic list`如果启动正常，会发现`/scan`话题。

##### 4.3.3rviz中订阅雷达相关消息

启动 rviz

![](file://D:/ROS资料/ROS笔记_melodic/ROS05_SLAM导航/img/09_雷达_订阅消息.png?lastModify=1600684748 "09\_雷达\_订阅消息")

注意: Fixed Frame 设置需要参考 rplidar.launch 中设置的 frame\_id

Topic 一般设置为 /scan

Size 可以自由调整

