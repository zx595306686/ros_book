### 6.7.3 摄像头信息仿真以及显示

通过 Gazebo 模拟摄像头传感器，并在 Rviz 中显示摄像头数据。

**实现流程:**

摄像头仿真基本流程:

1. 已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加摄像头配置

2. 将此文件集成进xacro文件

3. 启动 Gazebo，使用 Rviz 显示摄像头信息

#### 1.Gazebo 仿真摄像头

##### 1.1 新建 Xacro 文件，配置摄像头传感器信息

```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
  <!-- 被引用的link -->
  <gazebo reference="camera">
    <!-- 类型设置为 camara -->
    <sensor type="camera" name="camera_node">
      <update_rate>30.0</update_rate> <!-- 更新频率 -->
      <!-- 摄像头基本信息设置 -->
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>1280</width>
          <height>720</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <!-- 核心插件 -->
      <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>/camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

##### 1.2 xacro 文件集成

将步骤1的 Xacro 文件集成进总的机器人模型文件，代码示例如下:

```xml
<!-- 组合小车底盘与传感器 -->
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_head.urdf.xacro" />
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
    <xacro:include filename="move.urdf.xacro" />
    <!-- 摄像头仿真的 xacro 文件 -->
    <xacro:include filename="my_sensors_camara.urdf.xacro" />
</robot>
```

##### 1.3启动仿真环境

编写launch文件，启动gazebo，此处略...

#### 2.Rviz 显示摄像头数据

执行 gazebo 并启动 Rviz,在 Rviz 中添加摄像头组件。![](/assets/rgb摄像头.PNG)![](/assets/15_camera仿真.png)

