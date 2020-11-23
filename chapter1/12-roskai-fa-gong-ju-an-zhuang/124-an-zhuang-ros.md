### 1.2.4 安装 ROS

Ubuntu 安装完毕后，就可以安装 ROS 操作系统了，大致步骤如下:

1. 配置ubuntu的软件和更新；

2. 设置安装源；

3. 设置key；

4. 安装；

5. 配置环境变量。

---

#### 1.配置ubuntu的软件和更新

配置ubuntu的软件和更新，允许安装不经认证的软件。

首先打开“软件和更新”对话框，具体可以在 Ubuntu 搜索按钮中搜索。

打开后按照下图进行配置（确保勾选了"restricted"， "universe，" 和 "multiverse."）

![](/assets/00ROS安装之ubuntu准备.png "00ROS安装之ubuntu准备")

#### 2.设置安装源

官方默认安装源:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

或来自国内清华的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

或来自国内中科大的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

PS:

1. 回车后,可能需要输入管理员密码
2. 建议使用国内资源，安装速度更快。

#### 3.设置key

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

#### 4.安装

首先需要更新 apt\(以前是 apt-get, 官方建议使用 apt 而非 apt-get\),apt 是用于从互联网仓库搜索、安装、升级、卸载软件或操作系统的工具。

```
sudo apt update
```

等待...

然后，再安装所需类型的 ROS:ROS 多个类型:**Desktop-Full**、**Desktop**、**ROS-Base**。这里介绍较为常用的Desktop-Full\(官方推荐\)安装: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

```
sudo apt install ros-noetic-desktop-full
```

等待......\(比较耗时\)

友情提示: 由于网络原因,导致连接超时，可能会安装失败，如下所示:![](/assets/09_安装异常.PNG "09\_安装异常")可以多次重复调用 更新 和 安装命令，直至成功。

#### 5.配置环境变量

配置环境变量，方便在任意 终端中使用 ROS。

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

#### 卸载

如果需要卸载ROS可以调用如下命令:

```
sudo apt-get remove ros-noetic-*
```

注意: 在 ROS 版本 noetic 中无需构建软件包的依赖关系，没有`rosdep`的相关安装与配置。

---

另请参考：[http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)。

