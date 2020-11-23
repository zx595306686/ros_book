### 4.6.1 rosrun设置参数

rosrun 在启动节点时，也可以设置参数:

**语法: **rosrun 包名 节点名称 \_参数名:=参数值

#### 1.设置参数

启动乌龟显示节点，并设置参数 A = 100

```
rosrun turtlesim turtlesim_node _A:=100
```

#### 2.运行

`rosparam list`查看节点信息,显示结果:

```
/turtlesim/A
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

结果显示，参数A前缀节点名称，也就是说rosrun执行设置参数参数名使用的是私有模式

