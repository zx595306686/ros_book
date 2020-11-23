#### 1.5.2 ROS文件系统相关命令

ROS 的文件系统本质上都还是操作系统文件，我们可以使用Linux命令来操作这些文件，不过，在ROS中为了更好的用户体验，ROS专门提供了一些类似于Linux的命令，这些命令较之于Linux原生命令，更为简介、高效。文件操作，无外乎就是增删改查与执行等操作，接下来，我们就从这五个维度，来介绍ROS文件系统的一些常用命令。

#### 1.增

catkin\_create\_pkg 自定义包名 依赖包 === 创建新的ROS功能包

sudo apt install xxx === 安装 ROS功能包

**比如:**相机功能包安装`sudo apt-get install ros-noetic-usb-cam`

#### 2.删

sudo apt purge xxx  ==== 删除某个功能包

#### 3.查

rospack list === 列出所有功能包

rospack find 包名 === 查找某个功能包是否存在，如果存在返回安装路径

roscd 包名 === 进入某个功能包

rosls 包名 === 列出某个包下的文件

apt search xxx === 所有某个功能包

#### 4.改

rosed 包名 文件名 === 修改功能包文件

需要安装 vim

**比如:**rosed turtlesim Color.msg

#### 5.执行

##### 5.1roscore

**roscore === **是 ROS 的系统先决条件节点和程序的集合， 必须运行 roscore 才能使 ROS 节点进行通信。

roscore 将启动:

* ros master

* ros 参数服务器

* rosout 日志节点

用法:

```
roscore
```

或\(指定端口号\)

```
roscore -p xxxx
```

##### 5.2rosrun

**rosrun 包名 可执行文件名 ** === 运行指定的ROS节点

**比如:**`rosrun turtlesim turtlesim_node`

##### 5.3roslaunch

**roslaunch 包名 launch文件名** === 执行某个包下的 launch 文件

