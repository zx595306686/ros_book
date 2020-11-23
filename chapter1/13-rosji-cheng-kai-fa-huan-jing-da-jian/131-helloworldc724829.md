### 1.3.2 HelloWorld\(C++版\)

本节内容基于1.3.1，假设你已经创建了ROS的工作空间，并且创建了ROS的功能包，那么就可以进入核心步骤了，使用C++编写程序实现：

#### 1.进入 ros 包的 src 目录编辑源文件

```
cd 自定义的包
```

C++源码实现\(文件名自定义\)

```cpp
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"hello");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;
    //控制台输出 hello world
    ROS_INFO("hello world!");

    return 0;
}
```

#### 2.编辑 ros 包下的 Cmakelist.txt文件

```cmake
add_executable(步骤3的源文件名
  src/步骤3的源文件名.cpp
)
target_link_libraries(步骤3的源文件名
  ${catkin_LIBRARIES}
)
```

#### 3.进入工作空间目录并编译

```
cd 自定义空间名称
catkin_make
```

生成 build devel ....

#### 4.执行

**先启动命令行1：**

```
roscore
```

**再启动命令行2：**

```shell
cd 工作空间
source ./devel/setup.bash
rosrun 包名 C++节点
```

命令行输出: HelloWorld!

**PS:**`source ~/工作空间/devel/setup.bash`可以添加进`.bashrc`文件，使用上更方便

添加方式1: 直接使用 gedit 或 vi 编辑 .bashrc 文件，最后添加该内容

添加方式2:`echo "source ~/工作空间/devel/setup.bash" >> ~/.bashrc`

