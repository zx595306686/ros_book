### 1.3.3 HelloWorld\(Python版\)

本节内容基于1.3.1，假设你已经创建了ROS的工作空间，并且创建了ROS的功能包，那么就可以进入核心步骤了，使用Python编写程序实现：

#### 1.进入 ros 包添加 scripts 目录并编辑 python 文件

```
cd ros包
mkdir scripts
```

新建 python 文件: \(文件名自定义\)

```py
#! /usr/bin/env python

"""
    Python 版 HelloWorld

"""
import rospy

if __name__ == "__main__":
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
```

#### 2.为 python 文件添加可执行权限

```
chmod +x 自定义文件名.py
```

#### 3.编辑 ros 包下的 CamkeList.txt 文件

```
catkin_install_python(PROGRAMS scripts/自定义文件名.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 4.进入工作空间目录并编译

```
cd 自定义空间名称
catkin_make
```

#### 5.进入工作空间目录并执行

**先启动命令行1：**

```
roscore
```

**再启动命令行2：**

```
cd 工作空间
source ./devel/setup.bash
rosrun 包名 自定义文件名.py
```

输出结果:`Hello World!!!!`

