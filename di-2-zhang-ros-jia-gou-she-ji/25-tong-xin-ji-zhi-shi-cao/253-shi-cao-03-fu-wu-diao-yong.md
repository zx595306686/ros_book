### 2.5.3 实操03\_服务调用

**需求描述:**编码实现向 turtlesim 发送请求，在乌龟显示节点的窗体指定位置生成一乌龟，这是一个服务请求操作。

**结果演示:**

![](/assets/02_案例2_生成小乌龟.PNG)

**实现分析:**

1. 首先，需要启动乌龟显示节点。
2. 要通过ROS命令，来获取乌龟生成服务的服务名称以及服务消息类型。
3. 编写服务请求节点，生成新的乌龟。

**实现流程:**

1. 通过ros命令获取服务与服务消息信息。
2. 编码实现服务请求节点。
3. 启动 roscore、turtlesim\_node 、乌龟生成节点，生成新的乌龟。

#### 1.服务名称与服务消息获取

**获取话题:**/spawn

```
rosservice list
```

**获取消息类型:**turtlesim/Spawn

```
rosservice type /spawn
```

**获取消息格式:**

```
rossrv info turtlesim/Spawn
```

**响应结果:**

```
float32 x
float32 y
float32 theta
string name
---
string name
```

##### 

#### 2.服务客户端实现

创建功能包需要依赖的功能包: roscpp rospy std\_msgs geometry\_msgs

**实现方案A:**C++

```cpp
/*
    生成一只小乌龟
    准备工作:
        1.服务话题 /spawn
        2.服务消息类型 turtlesim/Spawn
        3.运行前先启动 turtlesim_node 节点

    实现流程:
        1.包含头文件
          需要包含 turtlesim 包下资源，注意在 package.xml 配置
        2.初始化 ros 节点
        3.创建 ros 句柄
        4.创建 service 客户端
        5.等待服务启动
        6.发送请求
        7.处理响应

*/

#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"set_turtle");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 service 客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    // 5.等待服务启动
    // client.waitForExistence();
    ros::service::waitForService("/spawn");
    // 6.发送请求
    turtlesim::Spawn spawn;
    spawn.request.x = 1.0;
    spawn.request.y = 1.0;
    spawn.request.theta = 1.57;
    spawn.request.name = "my_turtle";
    bool flag = client.call(spawn);
    // 7.处理响应结果
    if (flag)
    {
        ROS_INFO("新的乌龟生成,名字:%s",spawn.response.name.c_str());
    } else {
        ROS_INFO("乌龟生成失败！！！");
    }


    return 0;
}
```

配置文件此处略

**实现方案B:**Python

```py
#! /usr/bin/env python
"""
    生成一只小乌龟
    准备工作:
        1.服务话题 /spawn
        2.服务消息类型 turtlesim/Spawn
        3.运行前先启动 turtlesim_node 节点

    实现流程:
        1.导包
          需要包含 turtlesim 包下资源，注意在 package.xml 配置
        2.初始化 ros 节点
        3.创建 service 客户端
        4.等待服务启动
        5.发送请求
        6.处理响应

"""

import rospy
from turtlesim.srv import Spawn,SpawnRequest,SpawnResponse

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("set_turtle_p")
    # 3.创建 service 客户端
    client = rospy.ServiceProxy("/spawn",Spawn)
    # 4.等待服务启动
    client.wait_for_service()
    # 5.发送请求
    req = SpawnRequest()
    req.x = 2.0
    req.y = 2.0
    req.theta = -1.57
    req.name = "my_turtle_p"
    try:
        response = client.call(req)
        # 6.处理响应
        rospy.loginfo("乌龟创建成功!，叫:%s",response.name)
    except expression as identifier:
        rospy.loginfo("服务调用失败")
```

权限设置以及配置文件此处略

#### 3.运行

首先，启动 roscore；

然后启动乌龟显示节点；

最后启动乌龟生成请求节点；

最终执行结果与演示结果类似。

