### 1.4.2 安装VScode

#### 1.下载

vscode 下载:[https://code.visualstudio.com/docs?start=true](https://code.visualstudio.com/docs?start=true)

历史版本下载链接: [https://code.visualstudio.com/updates](https://code.visualstudio.com/updates)

#### 2.vscode 安装与卸载

##### 2.1 安装

**方式1:**双击安装即可\(或右击选择安装\)

**方式2:**`sudo dpkg -i xxxx.deb`

##### 2.2 卸载

```
sudo dpkg --purge  code
```

#### 3.vscode 集成 ROS 插件

使用 VScode 开发 ROS 程序，需要先安装一些插件，常用插件如下:

![](file://D:/ROS课程/ROS讲义_Noetic/ROS01_概述与环境搭建/img/06vscode插件.PNG?lastModify=1594352429 "06vscode插件")

#### 4.vscode 使用\_基本配置

##### 4.1 创建 ROS 工作空间

```
mkdir -p xxx_ws/src(必须得有 src)
cd xxx_ws
catkin_make
```

##### 4.2 启动 vscode

进入 xxx\_ws 启动 vscode

```
cd xxx_ws
code .
```

##### 4.3 vscode 中编译 ros

快捷键 ctrl + shift + B 调用编译，选择:`catkin_make:build`

可以点击配置设置为默认，修改.vscode/tasks.json 文件

```json
{
// 有关 tasks.json 格式的文档，请参见
    // https://go.microsoft.com/fwlink/?LinkId=733558
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make:debug", //代表提示的描述性信息
            "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
            "command": "catkin_make",//这个是我们需要运行的命令
            "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
            "group": {"kind":"build","isDefault":true},
            "presentation": {
                "reveal": "always"//可选always或者silence，代表是否输出信息
            },
            "problemMatcher": "$msCompile"
        }
    ]
}
```

##### 4.4 创建 ROS 功能包

选定 src 右击 ---&gt; create catkin package

**设置包名 添加依赖**

![](file://D:/ROS课程/ROS讲义_Noetic/ROS01_概述与环境搭建/img/07vscode_新建ROS包.PNG?lastModify=1594352429 "07vscode\_新建ROS包")

##### 4.5 C++ 实现

**在功能包的 src 下新建 cpp 文件**

```cpp
/*
    控制台输出 HelloVSCode !!!

*/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //执行节点初始化
    ros::init(argc,argv,"HelloVSCode");

    //输出日志
    ROS_INFO("Hello VSCode!!!哈哈哈哈哈哈哈哈哈哈");
    return 0;
}
```

**PS1: 如果没有代码提示**

修改 .vscode/c\_cpp\_properties.json

设置 "cppStandard": "c++17"

**PS2: main 函数的参数不可以被 const 修饰**

**PS3: 当ROS\_\_INFO 终端输出有中文时，会出现乱码**

[INFO](#): ????????????????????????

解决办法：在函数开头加入下面代码的任意一句

```cpp
setlocale(LC_CTYPE, "zh_CN.utf8");
setlocale(LC_ALL, "");
```

##### 4.6 python 实现

在 功能包 下新建 scripts 文件夹，添加 python 文件，**并添加可执行权限**

```py
#! /usr/bin/env python
"""
    Python 版本的 HelloVScode，执行在控制台输出 HelloVScode
    实现:
    1.导包
    2.初始化 ROS 节点
    3.日志输出 HelloWorld


"""

import rospy # 1.导包

if __name__ == "__main__":

    rospy.init_node("Hello_Vscode_p")  # 2.初始化 ROS 节点
    rospy.loginfo("Hello VScode, 我是 Python ....")  #3.日志输出 HelloWorld
```



##### 4.7 配置 CMakeLists.txt

C++ 配置:

```
add_executable(节点名称
  src/C++源文件名.cpp
)
target_link_libraries(节点名称
  ${catkin_LIBRARIES}
)
```

Python 配置:

```
catkin_install_python(PROGRAMS scripts/自定义文件名.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

##### 4.8 编译执行

编译: ctrl + shift + B

执行: 和之前一致，只是可以在 VScode 中添加终端，首先执行:`source ./devel/setup.bash`

![](file://D:/ROS课程/ROS讲义_Noetic/ROS01_概述与环境搭建/img/08vscode_执行.PNG?lastModify=1594352429 "08vscode\_执行")PS:

如果不编译直接执行 python 文件，会抛出异常。

1.第一行解释器声明，可以使用绝对路径定位到 python3 的安装路径 \#! /usr/bin/python3，但是不建议

2.建议使用 \#!/usr/bin/env python 但是会抛出异常 : /usr/bin/env: “python”: 没有那个文件或目录

3.解决1: \#!/usr/bin/env python3 直接使用 python3 但存在问题: 不兼容之前的 ROS 相关 python 实现

4.解决2: 创建一个链接符号到 python 命令:`sudo ln -s /usr/bin/python3 /usr/bin/python`

#### 5.其他 IDE

ROS 开发可以使用的 IDE 还是比较多的，除了上述的 VScode，还有 Eclipse、QT、PyCharm、Roboware ....,详情可以参考官网介绍:[http://wiki.ros.org/IDEs](http://wiki.ros.org/IDEs)

QT Creator Plugin for ROS，参考教程:[https://ros-qtc-plugin.readthedocs.io/en/latest/](https://ros-qtc-plugin.readthedocs.io/en/latest/)

Roboware 参考:[http://www.roboware.me/\#/](http://www.roboware.me/#/)\(PS: Roboware 已经停更了，可惜....\)

