### 1.2.6 资料:其他ROS版本安装

我们的教程采用的是ROS的最新版本noetic，不过noetic较之于之前的ROS版本变动较大且部分功能包还未更新，因此如果有需要\(比如到后期实践阶段，由于部分重要的功能包还未更新，需要ROS降级\)，也会安装之前版本的ROS，在此，建议选用的版本是melodic或kinetic。

接下来，就以melodic为例演示ROS历史版本安装\(当然先要准备与melodic对应的Ubuntu18.04\):

#### 1.配置ubuntu的软件和更新

首先打开“软件和更新”对话框，打开后按照下图进行配置（确保你的"restricted"， "universe，" 和 "multiverse."前是打上勾的）

![](file://D:/ROS资料/ROS笔记_melodic/ROS01_概述与环境搭建/img/00ROS安装之ubuntu准备.png?lastModify=1600843610 "00ROS安装之ubuntu准备")

#### 2.**安装源**

官方默认安装源:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

或来自国内中科大的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

或来自国内清华的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

PS:回车后,可能需要输入管理员密码

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
sudo apt install ros-melodic-desktop-full
```

等待...

#### 5.环境设置

配置环境变量，方便在任意 终端中使用 ROS。

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 6.安装构建依赖

首先安装构建依赖的相关工具

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

然后安装rosdep\(可以安装系统依赖\)

```
sudo apt install python-rosdep
```

初始化rosdep

```
sudo rosdep init
rosdep update
```

---

**注意:**

当执行到最后 sudo rosdep init 是，可能会抛出异常;

**错误提示:**

ERROR: cannot download default sources list from:  
[https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list](https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list)  
 Website may be down.

**原因:**

境外资源被屏蔽

**解决思路:**

查询错误提示中域名的IP地址，然后修改 /etc/hosts 文件，添加域名与IP映射

**实现:**

1.访问域名查询网址:[https://site.ip138.com/](https://site.ip138.com/)

2.查询域名ip，搜索框中输入: raw.githubusercontent.com，自由复制一个查询到的IP

![](/assets/域名ip查询.PNG)

3.修改 /etc/hosts 文件，命令:

```
sudo gedit /etc/hosts
```

添加内容:151.101.76.133    raw.githubusercontent.com \(查询到的ip与域名\)，保存并退出。

![](/assets/hosts文件修改.PNG)

或者，也可以使用 vi 或 vim 修改。

4.重新执行rosdep初始化与更新命令，如果rosdep update 抛出异常，基本都是网络原因导致的\(建议使用手机热点\)，多尝试几次即可。

![](/assets/rosdep初始化成功.PNG)

![](/assets/rosdepupdate成功.PNG)

---

综上，历史版本的安装与noetic流程类似，只是多出了“构建功能包依赖关系”的步骤。

另请参考：[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

