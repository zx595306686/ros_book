### 8.7.2 控制系统实现\_树莓派安装ROS

在树莓派上搭建ROS环境需要两步实现:

1. 在树莓派上安装Ubuntu
2. 基于Ubuntu安装ROS

版本选择:

* Ubuntu选用18.04桌面版
* ROS选用melodic
* 树莓派选用3b或3b+

注意:树莓派最新版本虽然是树莓派4b，性能也有较大升级，但是经测试树莓派4b虽然可以安装Ubuntu18.04桌面版，但是较之于树莓派3b和3b+稍有复杂\(后期应该会优化\)，且在控制系统的设计架构中，树莓派只是充当数据采集与运动控制的角色，不需要执行复杂的计算任务，树莓派3b或3b+也足以胜任，综上，树莓派没有选用3b或3b+。

具体实现流程如下。

#### 1.Ubuntu安装

##### 1.1硬件准备

* 树莓派
* 读卡器
* TF卡\(建议16G以及以上\)
* 显示器
* 数据线
* 鼠标
* 键盘
* 网线

![](/assets/树莓派系统安装.jpg)

##### 1.2软件准备

1.Ubuntu18.04下载并解压，下载地址:[https://ubuntu-mate.org/download/](https://ubuntu.com/download/raspberry-pi)

![](/assets/ubuntu镜像.png)![](/assets/ubuntu下载.png)

2.win32 Disk Imager烧录软件下载并安装，下载地址:[https://sourceforge.net/projects/win32diskimager/](https://sourceforge.net/projects/win32diskimager/)

根据提示下载并安装

3.如果TF卡已有内容，在使用之前需要执行格式化，比如可以使用SD Card Formatter:

SD Card Formatter下载并安装，下载地址:[https://www.sdcard.org/downloads/formatter/](https://www.sdcard.org/downloads/formatter/)

##### 1.3系统烧录

1.将TF卡插入读卡器，读卡器插入计算机；

2.如果TF卡已有内容，请先格式化\(如无数据，此步骤略过\)；

![](/assets/格式化.png)

3.启动win32 Disk Imager，选择先行下载的Ubuntu18.04镜像并写入TF卡；

![](/assets/树莓派4b安装Ubuntu镜像烧录.PNG)

![](/assets/树莓派4b安装Ubuntu镜像烧录成功.PNG)

##### 1.4系统安装

1.系统启动以及登录

取下TF卡插入树莓派，**连接网线**，启动树莓派，启动时是命令行界面，登陆使用默认账号密码，

默认账号:ubuntu

默认密码:ubuntu

还需要根据提示修改密码。

更改密码后，系统安装完毕，不过此时是命令行式操作，下一步需要安装桌面。

2.桌面安装

为了安装方便，建议使用ssh远程登录

首先，调用命令: ifconfig 获取树莓派的 ip 地址；

然后，远程调用 ssh ubuntu@ip地址登录；

接下来，可以直接安装桌面，但是为了提高安装效率，建议更换下载源，使用国内资源:

阿里云源

```
deb https://mirrors.aliyun.com/ubuntu-ports/ disco main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ disco main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ disco-security main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ disco-security main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ disco-updates main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ disco-updates main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ disco-backports main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ disco-backports main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ disco-proposed main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ disco-proposed main restricted universe multiverse
```

中科大源

```
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ disco main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu-ports/ disco main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-updates main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-backports main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-security main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-security main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-proposed main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu-ports/ disco-proposed main restricted universe multiverse
```

清华源

```
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-updates main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-backports main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-security main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-security main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-proposed main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ disco-proposed main restricted universe multiverse
```

修改/etc/apt/sources.list文件，将上述资源的任意一个复制进文件。

```
sudo nano /etc/apt/sources.list
```

最后，安装桌面环境（可选择：xubuntu-desktop、lubuntu-desktop、kubuntu-desktop）

```
sudo apt-get install ubuntu-desktop
```

3.重启桌面安装完毕

4.同步时间

默认情况下，树莓派系统时间是格林威治时间，而我们处于东八区，相差八个小时，需要将时间，设置为北京时间。

在/etc/profile文件中增加一行**export TZ='CST-8' **，并使文件立即生效，执行命令:

**source /etc/profile**或者**. /etc/profile**。

#### 2.ROS安装

在树莓派上安装ROS与PC上安装流程类似:

##### 1.配置软件与更新

首先打开“软件和更新”对话框，具体可以在 Ubuntu 搜索按钮中搜索。打开并配置（确保勾选了"restricted"， "universe，" 和 "multiverse."），可参考PC实现。

##### 2.设置安装源

官方默认安装源:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

或来自国内中科大的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

或来自国内清华的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

PS:回车后,可能需要输入管理员密码

##### 3.设置key

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

##### 4.安装

首先需要更新 apt\(以前是 apt-get, 官方建议使用 apt 而非 apt-get\),apt 是用于从互联网仓库搜索、安装、升级、卸载软件或操作系统的工具。

```
sudo apt update
```

等待...

然后，再安装所需类型的 ROS:ROS 多个类型:**Desktop-Full**、**Desktop**、**ROS-Base**。由于在分布式架构中，树莓派担当角色较为简单，在此选择 Desktop 或 ROS-Base 安装

```
sudo apt install ros-melodic-desktop
```

##### 5.环境配置

配置环境变量，方便在任意 终端中使用 ROS。

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

##### 6.构建软件包的依赖关系

到目前为止，已经安装了运行核心ROS软件包所需的软件。要创建和管理您自己的ROS工作区，还需要安装其他常用依赖:

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

安装并初始化rosdep，在使用许多ROS工具之前，需要初始化rosdep。rosdep使您可以轻松地为要编译的源安装系统依赖:

```
sudo apt install python-rosdep
```

使用以下命令，可以初始化rosdep。

```
sudo rosdep init
rosdep update
```



