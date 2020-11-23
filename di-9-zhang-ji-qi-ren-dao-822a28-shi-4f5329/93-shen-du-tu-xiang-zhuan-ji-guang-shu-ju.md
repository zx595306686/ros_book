## 9.3 深度图像转激光数据

实体机器人深度图像转换成激光数据实现与仿真环境类似。

#### 1.launch文件

launch 文件示例如下:

```xml
<launch>
    <param name="/use_sim_time" value="false" />
    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan">
        <remap from="image" to="/camera/depth/image_raw" />
        <param name="output_frame_id" value="camera_link"  />
    </node>
</launch>
```

需要注意 output\_frame\_id 的设置，需要与深度相机的 frame\_id 一致，我们使用的硬件，默认是 camera\_link。

#### 2.执行

1.PC端，启动roscore；

2.机器人端，ssh远程登陆机器人后，只需要启动深度相机节点即可\(不要启动激光雷达\)：

```
roslaunch astra_camera astra.launch
```

3.PC端，执行转换节点launch文件；

4.PC端，启动 rviz 查看执行结果。

![](/assets/深度图像转雷达数据.PNG)

