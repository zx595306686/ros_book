### 5.1.5 坐标系关系查看

在机器人系统中，涉及的坐标系有多个，为了方便查看，ros 提供了专门的工具，可以用于生成显示坐标系关系的 pdf 文件，该文件包含树形结构的坐标系图谱。

#### 6.1准备

首先调用`rospack find tf2_tools`查看是否包含该功能包，如果没有，请使用如下命令安装:

```
sudo apt-get install ros-noetic-tf2-tools
```

#### 6.2使用

##### 6.2.1生成 pdf 文件

启动坐标系广播程序之后，运行如下命令:

```
rosrun tf2_tools view_frames.py
```

会产生类似于下面的日志信息:

```
[INFO] [1592920556.827549]: Listening to tf data during 5 seconds...
[INFO] [1592920561.841536]: Generating graph in frames.pdf file...
```

查看当前目录会生成一个 frames.pdf 文件

##### 6.2.2查看文件

可以直接进入目录打开文件，或者调用命令查看文件:`evince frames.pdf`

内如如图所示:![](/assets/12坐标变换.PNG)

