### 1.4.3 launch文件演示

#### 1.需求

**需求:**一个程序中可能需要启动多个节点，比如:ROS 内置的小乌龟案例，如果要控制乌龟运动，要启动多个窗口，分别启动 roscore、乌龟界面节点、键盘控制节点。如果每次都调用 rosrun 逐一启动，显然效率低下，如何优化?

**方案:**官方给出的优化策略是使用 launch 文件，可以一次性启动多个 ROS 节点。

#### 2.实现

1. 选定功能包右击 ---&gt; 添加 launch 文件夹

2. 选定 launch 文件夹右击 ---&gt; 添加 launch 文件

3. 编辑 launch 文件内容

   ```
   <launch>
       <node pkg="helloworld" type="demo_hello" name="hello" output="screen" />
       <node pkg="turtlesim" type="turtlesim_node" name="t1"/>
       <node pkg="turtlesim" type="turtle_teleop_key" name="key1" />
   </launch>
   ```

   * node ---&gt; 包含的某个节点

   * pkg -----&gt; 功能包

   * type ----&gt; 被运行的节点文件

   * name --&gt; 为节点命名

   * output-&gt; 设置日志的输出目标

4. 运行 launch 文件

   `roslaunch 包名 launch文件名`

5. 运行结果: 一次性启动了多个节点



