### 2.5.4 实操04\_参数设置

**需求描述: **修改turtlesim乌龟显示节点窗体的背景色，已知背景色是通过参数服务器的方式以 rgb 方式设置的。

**结果演示:**

![](/assets/03_案例3_改变背景色.PNG)

**实现分析:**

1. 首先，需要启动乌龟显示节点。
2. 要通过ROS命令，来获取参数服务器中设置背景色的参数。
3. 编写参数设置节点，修改参数服务器中的参数值。

**实现流程:**

1. 通过ros命令获取参数。
2. 编码实现服参数设置节点。
3. 启动 roscore、turtlesim\_node 与参数设置节点，查看运行结果。

#### 1.参数名获取

**获取参数列表:**

```
rosparam list
```

**响应结果:**

```
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

#### 2.参数修改

**实现方案A:**C++

```cpp
/*
    注意命名空间的使用。

*/
#include "ros/ros.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"haha");

    ros::NodeHandle nh("turtlesim");
    //ros::NodeHandle nh;

    // ros::param::set("/turtlesim/background_r",0);
    // ros::param::set("/turtlesim/background_g",0);
    // ros::param::set("/turtlesim/background_b",0);

    nh.setParam("background_r",0);
    nh.setParam("background_g",0);
    nh.setParam("background_b",0);


    return 0;
}
```

配置文件此处略

**实现方案B:**Python

```py
#! /usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("hehe")
    # rospy.set_param("/turtlesim/background_r",255)
    # rospy.set_param("/turtlesim/background_g",255)
    # rospy.set_param("/turtlesim/background_b",255)
    rospy.set_param("background_r",255)
    rospy.set_param("background_g",255)
    rospy.set_param("background_b",255)  # 调用时，需要传入 __ns:=xxx
```

权限设置以及配置文件此处略

#### 3.运行

首先，启动 roscore；

然后启动背景色设置节点；

最后启动乌龟显示节点；

最终执行结果与演示结果类似。

PS: 注意节点启动顺序，如果先启动乌龟显示节点，后启动背景色设置节点，那么颜色设置不会生效。

#### 4.其他设置方式

**方式1:修改小乌龟节点的背景色\(命令行实现\)**

```
rosparam set /turtlesim/background_b 自定义数值
```

```
rosparam set /turtlesim/background_g 自定义数值
```

```
rosparam set /turtlesim/background_r 自定义数值
```

修改相关参数后，重启 turtlesim\_node 节点，背景色就会发生改变了

**方式2:启动节点时，直接设置参数**

```
rosrun turtlesim turtlesim_node _background_b:=100 _background_g=0 _background_b=0
```

**方式3:通过launch文件传参**

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="set_bg" output="screen">
        <!-- launch 传参策略1 -->
        <!-- <param name="background_b" value="0" type="int" />
        <param name="background_g" value="0" type="int" />
        <param name="background_r" value="0" type="int" /> -->

        <!-- launch 传参策略2 -->
        <rosparam command="load" file="$(find demo03_test_parameter)/cfg/color.yaml" />
    </node>

</
```

##### 



