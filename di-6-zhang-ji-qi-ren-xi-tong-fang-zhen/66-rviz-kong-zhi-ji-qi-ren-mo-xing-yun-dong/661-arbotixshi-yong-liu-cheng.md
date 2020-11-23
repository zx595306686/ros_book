### 6.5.1 Arbotix使用流程

接下来，通过一个案例演示 arbotix 的使用。

**需求描述:**

控制机器人模型在 rviz 中做圆周运动

**结果演示:**

![](/assets/arbotix运动控制.gif)

**实现流程:**

1. 安装 Arbotix

2. 创建新功能包，准备机器人 urdf、xacro 文件

3. 添加 Arbotix 配置文件

4. 编写 launch 文件配置 Arbotix

5. 启动 launch 文件并控制机器人模型运动

#### 1.安装 Arbotix

**方式1:**命令行调用

```
sudo apt-get install ros-<<VersionName()>>-arbotix
```

将 &lt;&lt;VsersionName\(\)&gt;&gt; 替换成当前 ROS 版本名称

**方式2:**源码安装

先从 github 下载源码，然后调用 catkin\_make 编译

```
git clone https://github.com/vanadiumlabs/arbotix_ros.git
```

#### 2.创建新功能包，准备机器人 urdf、xacro

urdf 和 xacro 调用上一讲实现即可

#### 3.添加 arbotix 所需的配置文件

**添加 arbotix 所需配置文件**

```yaml
# 该文件是控制器配置,一个机器人模型可能有多个控制器，比如: 底盘、机械臂、夹持器(机械手)....
# 因此，根 name 是 controller
controllers: {
   # 单控制器设置
   base_controller: {
          #类型: 差速控制器
       type: diff_controller,
       #参考坐标
       base_frame_id: base_footprint, 
       #两个轮子之间的间距
       base_width: 0.2,
       #控制频率
       ticks_meter: 2000, 
       #PID控制参数，使机器人车轮快速达到预期速度
       Kp: 12, 
       Kd: 12, 
       Ki: 0, 
       Ko: 50, 
       #加速限制
       accel_limit: 1.0 
    }
}
```

#### 4.launch 文件中配置 arbotix 节点

**launch 添加代码**

```xml
<node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
     <rosparam file="$(find my_urdf05_rviz)/config/hello.yaml" command="load" />
     <param name="sim" value="true" />
</node>
```

代码解释:

&lt;node&gt; 调用了 arbotix\_python 功能包下的 arbotix\_driver 节点

&lt;rosparam&gt; arbotix 驱动机器人运行时，需要获取机器人信息，可以通过 file 加载配置文件

&lt;param&gt; 在仿真环境下，需要配置 sim 为 true

#### 5.启动 launch 文件并控制机器人模型运动

**启动launch:**roslaunch xxxx ....launch

**配置 rviz:**

![](/assets/06_arbotix实现01.png)

**控制小车运动:**

此时调用 rostopic list 会发现一个熟悉的话题: /cmd\_vel![](/assets/07_arbotix实现02.png)也就说我们可以发布 cmd\_vel 话题消息控制小陈运动了，该实现策略有多种，可以另行编写节点，或者更简单些可以直接通过如下命令发布消息:

```
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```

现在，小车就可以运动起来了

