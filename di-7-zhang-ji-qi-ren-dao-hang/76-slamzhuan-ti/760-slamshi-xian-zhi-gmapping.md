### 7.6.0 SLAM实现之gmapping

gmapping 通过深度相机实现SLAM，将深度相机数据转换成雷达数据

1. 安装 
2. 启动深度相机

3. 执行

4. 查看

#### 1.安装

```
sudo apt-get install ros-melodic-depthimage-to-laserscan
```

#### 2.启动深度相机

在gazebo仿真中启动深度相机

#### 3.执行

编写launch文件执行，将深度信息转换成雷达信息

```xml
<launch>
    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan">
        <remap from="image" to="/camera/depth/image_raw" />
        <param name="output_frame_id" value="camera"  />
    </node>
</launch>
```

#### 4.查看

在rviz中查看雷达信息

