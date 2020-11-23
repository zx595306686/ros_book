### 1.2.3 安装ubuntu

#### 1.ubuntu安装

首先下载 Ubuntu 的镜像文件，链接如下:[http://mirrors.aliyun.com/ubuntu-releases/20.04/](http://mirrors.aliyun.com/ubuntu-releases/20.04/)；

然后，配置虚拟主机，关联 Ubuntu 镜像文件：![](/assets/27_Ubuntu安装01.png "27\_Ubuntu安装01")

![](/assets/28_Ubuntu安装02.png "28\_Ubuntu安装02")

![](/assets/29_Ubuntu安装03.png "29\_Ubuntu安装03")

启动后，开始配置 ubuntu 操作系统：

![](/assets/30_Ubuntu安装04.png "30\_Ubuntu安装04")

![](/assets/31_Ubuntu安装05.png "31\_Ubuntu安装05")安装过程中，断开网络连接，可以提升安装速度：

![](/assets/32_Ubuntu安装06.png "d")

![](/assets/33_Ubuntu安装07.png "33\_Ubuntu安装07")

![](/assets/34_Ubuntu安装08.png "34\_Ubuntu安装08")

![](/assets/35_Ubuntu安装09.png "35\_Ubuntu安装09")

![](/assets/36_Ubuntu安装10.png "36\_Ubuntu安装10")

![](/assets/37_Ubuntu安装11.png "37\_Ubuntu安装11")

安装完毕后，会给出重启提示，点击重启确定按钮即可：

![](/assets/38_Ubuntu安装12.png "38\_Ubuntu安装12")

到目前为止 VirtualBox 已经正常安装了 ubuntu, 并启动成功。

#### 2.使用优化

为了优化 ubuntu 操作的用户体验，方便虚拟机与宿主机的文件交换以及 USB 设备的正常使用，还需做如下操作:

##### 1.安装虚拟机工具

![](/assets/40_Ubuntu优化02.png "40\_Ubuntu优化02")

![](/assets/41_Ubuntu优化03.png "41\_Ubuntu优化03")

重启使之生效，选择菜单栏的`自动调整窗口大小`,然后ubuntu 桌面会自动使用窗口大小:`右ctrl + F`全屏。

##### 2.启动文件交换模式

![](/assets/39_Ubuntu优化01.png "39\_Ubuntu优化01")

##### 3.安装扩展插件

**先去 virtualbox 官网下载扩展包**

![](/assets/14_virtualbox下载.png "14\_virtualbox下载")

**在 virtual box 中添加扩展工具**

![](/assets/42_Ubuntu优化04.png "42\_Ubuntu优化04")

**在虚拟机中添加 USB 设备**

![](/assets/43_Ubuntu优化05.png "43\_Ubuntu优化05")

重启后，使用`ll /dev/ttyUSB* 或 ll /dev/ttyACM*`即可查看新接入的设备。

##### 4.其他

其他设置，比如输入法可以根据喜好自行下载安装。

ubuntu 20.04 鼠标右击没有创建文件选项，如果想要设置此选项，可以进入`主目录`下的`模板`目录，使用 gedit 创建一个空文本文档，以后，鼠标右击就可以添加新建文档选项，并且创建的文档与当前自定义的文档名称一致

....

