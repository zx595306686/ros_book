### 5.1.6 TF坐标变换实操

**需求描述:**

程序启动之初: 产生两只乌龟，中间的乌龟\(A\) 和 左下乌龟\(B\), B 会自动运行至A的位置，并且键盘控制时，只是控制 A 的运动，但是 B 可以跟随 A 运行

**结果演示:**

**实现分析:**

乌龟跟随实现的核心，是乌龟A和B都要发布相对世界坐标系的坐标信息，然后，订阅到该信息需要转换获取A相对于B坐标系的信息，最后，再生成速度信息，并控制B运动。

1. 启动乌龟显示节点
2. 在乌龟显示窗体中生成一只新的乌龟\(需要使用服务\)
3. 编写两只乌龟发布坐标信息的节点
4. 编写订阅节点订阅坐标信息并生成新的相对关系生成速度信息

**实现流程:**C++ 与 Python 实现流程一致

1. 新建功能包，添加依赖

2. 编写服务客户端，用于生成一只新的乌龟

3. 编写发布方，发布两只乌龟的坐标信息

4. 编写订阅方，订阅两只乌龟信息，生成速度信息并发布

5. 运行

**准备工作:**

1.了解如何创建第二只乌龟，且不受键盘控制

创建第二只乌龟需要使用rosservice,话题使用的是 spawn

```
rosservice call /spawn "x: 1.0
y: 1.0
theta: 1.0
name: 'turtle_flow'" 
name: "turtle_flow"
```

键盘是无法控制第二只乌龟运动的，因为使用的话题: /第二只乌龟名称/cmd\_vel,对应的要控制乌龟运动必须发布对应的话题消息

2.了解如何获取两只乌龟的坐标

是通过话题 /乌龟名称/pose 来获取的

```
x: 1.0 //x坐标
y: 1.0 //y坐标
theta: -1.21437060833 //角度
linear_velocity: 0.0 //线速度
angular_velocity: 1.0 //角速度
```

---

方案A:C++实现

#### 1.创建功能包

创建项目功能包依赖于 tf2、tf2\_ros、tf2\_geometry\_msgs、roscpp rospy std\_msgs geometry\_msgs、turtlesim

#### 2.服务客户端\(生成乌龟\)

```cpp
/* 
    创建第二只小乌龟
 */
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{

    setlocale(LC_ALL,"");

    //执行初始化
    ros::init(argc,argv,"create_turtle");
    //创建节点
    ros::NodeHandle nh;
    //创建服务客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    ros::service::waitForService("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle2";
    spawn.request.x = 1.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 3.12415926;
    bool flag = client.call(spawn);
    if (flag)
    {
        ROS_INFO("乌龟%s创建成功!",spawn.response.name.c_str());
    }
    else
    {
        ROS_INFO("乌龟2创建失败!");
    }

    ros::spin();

    return 0;
}
```

配置文件此处略。

#### 3.发布方\(发布两只乌龟的坐标信息\)

可以订阅乌龟的位姿信息，然后再转换成坐标信息，两只乌龟的实现逻辑相同，只是订阅的话题名称，生成的坐标信息等稍有差异，可以将差异部分通过参数传入:

* 该节点需要启动两次
* 每次启动时都需要传入乌龟节点名称\(第一次是 turtle1 第二次是 turtle2\)

```cpp
/*  
    该文件实现:需要订阅 turtle1 和 turtle2 的 pose，然后广播相对 world 的坐标系信息

    注意: 订阅的两只 turtle,除了命名空间(turtle1 和 turtle2)不同外,
          其他的话题名称和实现逻辑都是一样的，
          所以我们可以将所需的命名空间通过 args 动态传入

    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.解析传入的命名空间
        4.创建 ros 句柄
        5.创建订阅对象
        6.回调函数处理订阅的 pose 信息
            6-1.创建 TF 广播器
            6-2.将 pose 信息转换成 TransFormStamped
            6-3.发布
        7.spin

*/
//1.包含头文件
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
//保存乌龟名称
std::string turtle_name;


void doPose(const turtlesim::Pose::ConstPtr& pose){
    //  6-1.创建 TF 广播器 ---------------------------------------- 注意 static
    static tf2_ros::TransformBroadcaster broadcaster;
    //  6-2.将 pose 信息转换成 TransFormStamped
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = turtle_name;
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    //  6-3.发布
    broadcaster.sendTransform(tfs);

} 

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"pub_tf");
    // 3.解析传入的命名空间
    if (argc != 2)
    {
        ROS_ERROR("请传入正确的参数");
    } else {
        turtle_name = argv[1];
        ROS_INFO("乌龟 %s 坐标发送启动",turtle_name.c_str());
    }

    // 4.创建 ros 句柄
    ros::NodeHandle nh;
    // 5.创建订阅对象
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(turtle_name + "/pose",1000,doPose);
    //     6.回调函数处理订阅的 pose 信息
    //         6-1.创建 TF 广播器
    //         6-2.将 pose 信息转换成 TransFormStamped
    //         6-3.发布
    // 7.spin
    ros::spin();
    return 0;
}
```

配置文件此处略。

#### 4.订阅方\(解析坐标信息并生成速度信息\)

```cpp
/*  
    订阅 turtle1 和 turtle2 的 TF 广播信息，查找并转换时间最近的 TF 信息
    将 turtle1 转换成相对 turtle2 的坐标，在计算线速度和角速度并发布

    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.创建 ros 句柄
        4.创建 TF 订阅对象
        5.处理订阅到的 TF
        6.spin

*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"sub_TF");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 TF 订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    // 5.处理订阅到的 TF

    // 需要创建发布 /turtle2/cmd_vel 的 publisher 对象

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",1000);

    ros::Rate rate(10);
    while (ros::ok())
    {
        try
        {
            //5-1.先获取 turtle1 相对 turtle2 的坐标信息
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));

            //5-2.根据坐标信息生成速度信息 -- geometry_msgs/Twist.h
            geometry_msgs::Twist twist;
            twist.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x,2) + pow(tfs.transform.translation.y,2));
            twist.angular.z = 4 * atan2(tfs.transform.translation.y,tfs.transform.translation.x);

            //5-3.发布速度信息 -- 需要提前创建 publish 对象
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("错误提示:%s",e.what());
        }



        rate.sleep();
        // 6.spin
        ros::spinOnce();
    }

    return 0;
}
```

配置文件此处略。

#### 5.运行

使用 launch 文件组织需要运行的节点，内容示例如下:

```xml
<!--
    tf2 实现小乌龟跟随案例
-->
<launch>
    <!-- 启动乌龟节点与键盘控制节点 -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key" name="key_control" output="screen"/>
    <!-- 启动创建第二只乌龟的节点 -->
    <node pkg="demo_tf2_test" type="Test01_Create_Turtle2" name="turtle2" output="screen" />
    <!-- 启动两个坐标发布节点 -->
    <node pkg="demo_tf2_test" type="Test02_TF2_Caster" name="caster1" output="screen" args="turtle1" />
    <node pkg="demo_tf2_test" type="Test02_TF2_Caster" name="caster2" output="screen" args="turtle2" />
    <!-- 启动坐标转换节点 -->
    <node pkg="demo_tf2_test" type="Test03_TF2_Listener" name="listener" output="screen" />
</launch>
```

---

方案B:Python实现

#### 1.创建功能包

创建项目功能包依赖于 tf2、tf2\_ros、tf2\_geometry\_msgs、roscpp rospy std\_msgs geometry\_msgs、turtlesim

#### 2.服务客户端\(生成乌龟\)

```py
#! /usr/bin/env python
"""  
    调用 service 服务在窗体指定位置生成一只乌龟
    流程:
        1.导包
        2.初始化 ros 节点
        3.创建服务客户端
        4.等待服务启动
        5.创建请求数据
        6.发送请求并处理响应
"""
#1.导包
import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("turtle_spawn_p")
    # 3.创建服务客户端
    client = rospy.ServiceProxy("/spawn",Spawn)
    # 4.等待服务启动
    client.wait_for_service()
    # 5.创建请求数据
    req = SpawnRequest()
    req.x = 1.0
    req.y = 1.0
    req.theta = 3.14
    req.name = "turtle2"
    # 6.发送请求并处理响应
    try:
        response = client.call(req)
        rospy.loginfo("乌龟创建成功，名字是:%s",response.name)
    except Exception as e:
        rospy.loginfo("服务调用失败....")
```

权限设置以及配置文件此处略。

#### 3.发布方\(发布两只乌龟的坐标信息\)

```py
#! /usr/bin/env python
"""  
    该文件实现:需要订阅 turtle1 和 turtle2 的 pose，然后广播相对 world 的坐标系信息

    注意: 订阅的两只 turtle,除了命名空间(turtle1 和 turtle2)不同外,
          其他的话题名称和实现逻辑都是一样的，
          所以我们可以将所需的命名空间通过 args 动态传入

    实现流程:
        1.导包
        2.初始化 ros 节点
        3.解析传入的命名空间
        4.创建订阅对象
        5.回调函数处理订阅的 pose 信息
            5-1.创建 TF 广播器
            5-2.将 pose 信息转换成 TransFormStamped
            5-3.发布
        6.spin
"""
# 1.导包
import rospy
import sys
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions

turtle_name = ""

def doPose(pose):
    # rospy.loginfo("x = %.2f",pose.x)
    #1.创建坐标系广播器
    broadcaster = tf2_ros.TransformBroadcaster()
    #2.将 pose 信息转换成 TransFormStamped
    tfs = TransformStamped()
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()

    tfs.child_frame_id = turtle_name
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0.0

    qtn = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]

    #3.广播器发布 tfs
    broadcaster.sendTransform(tfs)


if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("sub_tfs_p")
    # 3.解析传入的命名空间
    rospy.loginfo("-------------------------------%d",len(sys.argv))
    if len(sys.argv) < 2:
        rospy.loginfo("请传入参数:乌龟的命名空间")
    else:
        turtle_name = sys.argv[1]
    rospy.loginfo("///////////////////乌龟:%s",turtle_name)

    rospy.Subscriber(turtle_name + "/pose",Pose,doPose)
    #     4.创建订阅对象
    #     5.回调函数处理订阅的 pose 信息
    #         5-1.创建 TF 广播器
    #         5-2.将 pose 信息转换成 TransFormStamped
    #         5-3.发布
    #     6.spin
    rospy.spin()
```

权限设置以及配置文件此处略。

#### 4.订阅方\(解析坐标信息并生成速度信息\)

```py
#! /usr/bin/env python
"""  
    订阅 turtle1 和 turtle2 的 TF 广播信息，查找并转换时间最近的 TF 信息
    将 turtle1 转换成相对 turtle2 的坐标，在计算线速度和角速度并发布

    实现流程:
        1.导包
        2.初始化 ros 节点
        3.创建 TF 订阅对象
        4.处理订阅到的 TF
            4-1.查找坐标系的相对关系
            4-2.生成速度信息，然后发布
"""
# 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import math

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("sub_tfs_p")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    # 4.处理订阅到的 TF
    rate = rospy.Rate(10)
    # 创建速度发布对象
    pub = rospy.Publisher("/turtle2/cmd_vel",Twist,queue_size=1000)
    while not rospy.is_shutdown():

        rate.sleep()
        try:
            #def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            trans = buffer.lookup_transform("turtle2","turtle1",rospy.Time(0))
            # rospy.loginfo("相对坐标:(%.2f,%.2f,%.2f)",
            #             trans.transform.translation.x,
            #             trans.transform.translation.y,
            #             trans.transform.translation.z
            #             )   
            # 根据转变后的坐标计算出速度和角速度信息
            twist = Twist()
            # 间距 = x^2 + y^2  然后开方
            twist.linear.x = 0.5 * math.sqrt(math.pow(trans.transform.translation.x,2) + math.pow(trans.transform.translation.y,2))
            twist.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)

            pub.publish(twist)

        except Exception as e:
            rospy.logwarn("警告:%s",e)
```

权限设置以及配置文件此处略。

#### 5.运行

使用 launch 文件组织需要运行的节点，内容示例如下:

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key" name="key_control" output="screen"/>

    <node pkg="demo06_test_flow_p" type="test01_turtle_spawn_p.py" name="turtle_spawn" output="screen"/>

    <node pkg="demo06_test_flow_p" type="test02_turtle_tf_pub_p.py" name="tf_pub1" args="turtle1" output="screen"/>
    <node pkg="demo06_test_flow_p" type="test02_turtle_tf_pub_p.py" name="tf_pub2" args="turtle2" output="screen"/>
    <node pkg="demo06_test_flow_p" type="test03_turtle_tf_sub_p.py" name="tf_sub" output="screen"/>

</launch>
```



