### 8.6.6 基于ros\_arduino\_bridge的底盘实现\_05ROS端配置

ROS端的实现相对要简单很多，只需要修改配置文件即可，具体实现如下:

1. 复制并修改配置文件
2. 启动launch文件并测试

#### 1.配置文件

ROS端功能包是 ros\_arduino\_python，程序入口是该包launch目录下的arduino.launch文件，内容如下:

```xml
<launch>
   <node name="arduino" pkg="ros_arduino_python" type="arduino_node.py" output="screen">
      <rosparam file="$(find ros_arduino_python)/config/my_arduino_params.yaml" command="load" />
   </node>
</launch>
```

需要载入yaml格式的配置文件，该文件在 config 目录下已经提供了模板，只需要复制文件并按需配置即可，复制文件并重命名，配置如下：

```yaml
# For a direct USB cable connection, the port name is typically
# /dev/ttyACM# where is # is a number such as 0, 1, 2, etc
# For a wireless connection like XBee, the port is typically
# /dev/ttyUSB# where # is a number such as 0, 1, 2, etc.

port: /dev/ttyACM0 #视情况设置，一般设置为 /dev/ttyACM0 或 /dev/ttyUSB0
baud: 57600 #波特率
timeout: 0.1 #超时时间

rate: 50
sensorstate_rate: 10

use_base_controller: True  #启用基座控制器
base_controller_rate: 10   

# For a robot that uses base_footprint, change base_frame to base_footprint
base_frame: base_footprint #base_frame 设置

# === Robot drivetrain parameters
wheel_diameter: 0.065 #车轮直径
wheel_track: 0.20 #轮间距
encoder_resolution: 3120 #编码器精度(一圈的脉冲数 * 倍频 * 减速比)
#gear_reduction: 1 #减速比
#motors_reversed: False #转向取反

# === PID parameters PID参数，需要自己调节
Kp: 5
Kd: 45
Ki: 0
Ko: 50
accel_limit: 1.0

# === Sensor definitions.  Examples only - edit for your robot.
#     Sensor type can be one of the follow (case sensitive!):
#      * Ping
#      * GP2D12
#      * Analog
#      * Digital
#      * PololuMotorCurrent
#      * PhidgetsVoltage
#      * PhidgetsCurrent (20 Amp, DC)



sensors: {
  #motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  #motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  #ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  #sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  arduino_led:          {pin: 13, type: Digital, rate: 5, direction: output}
}
```

#### 2.测试

启动launch文件、rviz与键盘控制节点，rviz中可以添加TF与Odometry组件\(将Global Option 与 Odometry 的 Topic 设置为 odom\)，最终运行结果与8.5.7类似。

