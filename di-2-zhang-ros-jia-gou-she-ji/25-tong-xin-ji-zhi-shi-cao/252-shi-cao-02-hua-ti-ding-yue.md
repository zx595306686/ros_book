### 2.5.2 实操02\_话题订阅

**需求描述: **已知turtlesim中的乌龟显示节点，会发布当前乌龟的位姿\(窗体中乌龟的坐标以及朝向\)，要求控制乌龟运动，并时时打印当前乌龟的位姿。

**结果演示:**

![](/assets/案例02_乌龟位姿.gif)

**实现分析:**

1. 首先，需要启动乌龟显示以及运动控制节点并控制乌龟运动。
2. 要通过ROS命令，来获取乌龟位姿发布的话题以及消息。
3. 编写订阅节点，订阅并打印乌龟的位姿。

**实现流程:**

1. 通过ros命令获取话题与消息信息。
2. 编码实现位姿获取节点。
3. 启动 roscore、turtlesim\_node 、控制节点以及位姿订阅节点，控制乌龟运动并输出乌龟的位姿。

#### 1.话题与消息获取

**获取话题:**/turtle1/pose

```
rostopic list
```

**获取消息类型:**turtlesim/Pose

```
rostopic type  /turtle1/pose
```

**获取消息格式:**

```
rosmsg info turtlesim/Pose
```

**响应结果:**

```
​float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

#### 2.实现订阅节点

创建功能包需要依赖的功能包: roscpp rospy std\_msgs turtlesim

**实现方案A: **C++

```cpp
/*  
    订阅小乌龟的位姿: 时时获取小乌龟在窗体中的坐标并打印
    准备工作:
        1.获取话题名称 /turtle1/pose
        2.获取消息类型 turtlesim/Pose
        3.运行前启动 turtlesim_node 与 turtle_teleop_key 节点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建订阅者对象
        5.回调函数处理订阅的数据
        6.spin
*/

#include "ros/ros.h"
#include "turtlesim/Pose.h"

void doPose(const turtlesim::Pose::ConstPtr& p){
    ROS_INFO("乌龟位姿信息:x=%.2f,y=%.2f,theta=%.2f,lv=%.2f,av=%.2f",
        p->x,p->y,p->theta,p->linear_velocity,p->angular_velocity
    );
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"sub_pose");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅者对象
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1000,doPose);
    // 5.回调函数处理订阅的数据
    // 6.spin
    ros::spin();
    return 0;
}
```

配置文件此处略

**实现方案B: **Python

```py
#! /usr/bin/env python
"""
    订阅小乌龟的位姿: 时时获取小乌龟在窗体中的坐标并打印
    准备工作:
        1.获取话题名称 /turtle1/pose
        2.获取消息类型 turtlesim/Pose
        3.运行前启动 turtlesim_node 与 turtle_teleop_key 节点

    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建订阅者对象
        4.回调函数处理订阅的数据
        5.spin

"""

import rospy
from turtlesim.msg import Pose

def doPose(data):
    rospy.loginfo("乌龟坐标:x=%.2f, y=%.2f,theta=%.2f",data.x,data.y,data.theta)

if __name__ == "__main__":

    # 2.初始化 ROS 节点
    rospy.init_node("sub_pose_p")

    # 3.创建订阅者对象
    sub = rospy.Subscriber("/turtle1/pose",Pose,doPose,queue_size=1000)
    #     4.回调函数处理订阅的数据
    #     5.spin
    rospy.spin()
```

权限设置以及配置文件此处略

#### 3.运行

首先，启动 roscore；

然后启动乌龟显示节点，执行运动控制节点；

最后启动乌龟位姿订阅节点；

最终执行结果与演示结果类似。

