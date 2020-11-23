### 5.1.2 静态坐标变换

所谓静态坐标变换，是指两个坐标系之间的相对位置是固定的。

**需求描述:**

现有一机器人模型，核心构成包含主体与雷达，各对应一坐标系，坐标系的原点分别位于主体与雷达的物理中心，已知雷达原点相对于主体原点位移关系如下: x 0.1 y0.0 z0.2。当前雷达检测到一障碍物，在雷达坐标系中障碍物的坐标为 \(0.3 0.0 0.0\),请问，该障碍物相对于主体的坐标是多少？

**结果演示:**

**实现分析:**

1. 实现坐标变换，必须需要坐标系的相对关系以及坐标点信息
2. 坐标系相对关系，可以通过发布方发布
3. 订阅方，订阅到发布的坐标系相对关系，再传入坐标点信息\(可以写死\)，然后借助于 tf 实现坐标变换，并将结果输出

**实现流程:**C++ 与 Python 实现流程一致

1. 新建功能包，添加依赖
2. 编写发布方实现
3. 编写订阅方实现
4. 执行并查看结果

---

方案A:C++实现

#### 1.创建功能包

创建项目功能包依赖于 tf2、tf2\_ros、tf2\_geometry\_msgs、roscpp rospy std\_msgs geometry\_msgs

#### 2.发布方

```cpp
/* 
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建静态坐标转换广播器
        4.创建坐标系信息
        5.广播器发布坐标系信息
        6.spin()
*/


// 1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"static_brocast");
    // 3.创建静态坐标转换广播器
    tf2_ros::StaticTransformBroadcaster broadcaster;
    // 4.创建坐标系信息
    geometry_msgs::TransformStamped ts;
    //----设置头信息
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";
    //----设置子级坐标系
    ts.child_frame_id = "laser";
    //----设置子级相对于父级的偏移量
    ts.transform.translation.x = 0.2;
    ts.transform.translation.x = 0.0;
    ts.transform.translation.x = 0.5;
    //----设置四元数:将 欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    // 5.广播器发布坐标系信息
    broadcaster.sendTransform(ts);
    ros::spin();
    return 0;
}
```

配置文件此处略。

#### 3.订阅方

```cpp
/*  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin()
*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"tf_sub");
    ros::NodeHandle nh;
    // 3.创建 TF 订阅节点
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1;
        point_laser.point.y = 2;
        point_laser.point.z = 7.3;
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");
            ROS_INFO("坐标点相对于 base_link 的坐标为:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }


        r.sleep();  
        ros::spinOnce();
    }


    return 0;
}
```

配置文件此处略。

#### 4.执行

可以使用命令行或launch文件的方式分别启动发布节点与订阅节点，如果程序无异常，控制台将输出，坐标转换后的结果。

---

方案B:Python实现

#### 1.创建功能包

创建项目功能包依赖于 tf2、tf2\_ros、tf2\_geometry\_msgs、roscpp rospy std\_msgs geometry\_msgs

#### 2.发布方

```py
#! /usr/bin/env python
"""  
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 静态坐标广播器
        4.创建并组织被广播的消息
        5.广播器发送消息
        6.spin
"""
# 1.导包
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("static_tf_pub_p")
    # 3.创建 静态坐标广播器
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # 4.创建并组织被广播的消息
    tfs = TransformStamped()
    # --- 头信息
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    tfs.header.seq = 101
    # --- 子坐标系
    tfs.child_frame_id = "radar"
    # --- 坐标系相对信息
    # ------ 偏移量
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5
    # ------ 四元数
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]


    # 5.广播器发送消息
    broadcaster.sendTransform(tfs)
    # 6.spin
    rospy.spin()
```

权限设置以及配置文件此处略。

#### 3.订阅方

```py
#! /usr/bin/env python
"""  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，
    转换成父级坐标系中的坐标点

    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.创建一个 radar 坐标系中的坐标点
        5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 world 的坐标
        6.spin

"""
# 1.导包
import rospy
import tf2_ros
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
# from geometry_msgs.msg import PointStamped

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("static_sub_tf_p")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():    
    # 4.创建一个 radar 坐标系中的坐标点
        point_source = PointStamped()
        point_source.header.frame_id = "radar"
        point_source.header.stamp = rospy.Time.now()
        point_source.point.x = 10
        point_source.point.y = 2
        point_source.point.z = 3

        try:
    #     5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 world 的坐标
            point_target = buffer.transform(point_source,"world")
            rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                            point_target.point.x,
                            point_target.point.y,
                            point_target.point.z)
        except Exception as e:
            rospy.logerr("异常:%s",e)

    #     6.spin
        rate.sleep()
```

权限设置以及配置文件此处略。

**PS: 在 tf2 的 python 实现中，tf2 已经封装了一些消息类型，不可以使用 geometry\_msgs.msg 中的类型**

#### 4.执行

可以使用命令行或launch文件的方式分别启动发布节点与订阅节点，如果程序无异常，控制台将输出，坐标转换后的结果。

---

#### 5.补充

当坐标系之间的相对位置固定时，那么所需参数也是固定的: 父系坐标名称、子级坐标系名称、x偏移量、y偏移量、z偏移量、x 翻滚角度、y俯仰角度、z偏航角度，实现逻辑相同，参数不同，那么 ROS 系统就已经封装好了专门的节点，使用方式如下:

`rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系`

示例:`rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser`

也建议使用该种方式直接实现静态坐标系相对信息发布。

