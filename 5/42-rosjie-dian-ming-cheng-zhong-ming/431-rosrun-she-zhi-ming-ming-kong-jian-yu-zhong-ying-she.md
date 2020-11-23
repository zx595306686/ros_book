### 4.4.1 rosrun设置命名空间与重映射

#### 1.rosrun设置命名空间

##### 1.1设置命名空间演示

语法: rosrun 包名 节点名 \_\_ns:=新名称

```
rosrun turtlesim turtlesim_node __ns:=/xxx
```

```
rosrun turtlesim turtlesim_node __ns:=/yyy
```

两个节点都可以正常运行

##### 1.2运行结果

`rosnode list`查看节点信息,显示结果:

```
/xxx/turtlesim
/yyy/turtlesim
```

#### 2.rosrun名称重映射

##### 2.1为节点起别名

语法: rosrun 包名 节点名 \_\_name:=新名称

```
rosrun turtlesim  turtlesim_node __name:=t1 |  rosrun turtlesim   turtlesim_node /turtlesim:=t1(不适用于python)
```

```
rosrun turtlesim  turtlesim_node __name:=t2 |  rosrun turtlesim   turtlesim_node /turtlesim:=t2(不适用于python)
```

两个节点都可以运行

##### 2.2运行结果

`rosnode list`查看节点信息,显示结果:

```
/t1
/t2
```

#### 3.rosrun命名空间与名称重映射叠加

##### 3.1设置命名空间同时名称重映射

语法: rosrun 包名 节点名 \_\_ns:=新名称 \_\_name:=新名称

```
rosrun turtlesim turtlesim_node __ns:=/xxx __name:=tn
```

##### 3.2运行结果

`rosnode list`查看节点信息,显示结果:

```
/xxx/tn
```

> 使用环境变量也可以设置命名空间,启动节点前在终端键入如下命令:
>
> export ROS\_NAMESPACE=xxxx



