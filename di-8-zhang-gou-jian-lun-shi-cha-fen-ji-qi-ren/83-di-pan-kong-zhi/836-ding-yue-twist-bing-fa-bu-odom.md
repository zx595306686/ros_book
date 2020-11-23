### 8.5.7 程序设计策略A自实现\_05ROS端

到目前为止，驱动系统的代码已经编写完毕，Arduino端可以发布实际速度信息，接下来在ROS端我们需要订阅底盘速度信息，并根据速度信息生成里程计消息并发布，还需要生成坐标变换消息并发布。

ROS 端实现:

```cpp
/*
    需求：订阅 /car_vel 消息，生成 /odom 并发布
    实现:
        0.了解 nav_msgs/Odometry 与 geometry_msgs/TransformStamped 
        1.搭建框架，订阅 /car_vel 发布 /odom
        2.根据 /car_vel 生成 /odom,以及坐标变换

    0. nav_msgs/Odometry 消息格式
        调用命令:  rosmsg info nav_msgs/Odometry
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id //父级坐标系
        string child_frame_id //子级坐标系
        geometry_msgs/PoseWithCovariance pose
            geometry_msgs/Pose pose
                geometry_msgs/Point position //坐标点
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Quaternion orientation //四元数
                    float64 x
                    float64 y
                    float64 z
                    float64 w
            float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
            geometry_msgs/Twist twist
                geometry_msgs/Vector3 linear //线速度
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Vector3 angular //角速度
                    float64 x
                    float64 y
                    float64 z
            float64[36] covariance


        消息格式:geometry_msgs/TransformStamped
        调用命令: rosmsg info geometry_msgs/TransformStamped
        std_msgs/Header header
        uint32 seq
            time stamp
            string frame_id
        string child_frame_id
        geometry_msgs/Transform transform
            geometry_msgs/Vector3 translation
                float64 x
                float64 y
                float64 z
            geometry_msgs/Quaternion rotation
                float64 x
                float64 y
                float64 z
                float64 w


    1. 搭建框架，订阅 /car_vel 发布 /odom
        典型的订阅发布实现

    2.根据 /car_vel 生成 /odom
        根据小车线速度与角速度，计算单位时间内位移与角度，并累加计算出小车此刻的位置
        组织坐标信息并发布
        组织里程计信息并发布

*/
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

//保存小车当前坐标信息(初始位置为坐标系原点)
static double x = 0.0;
static double y = 0.0;
static double th = 0.0;

//接收订阅到的线速度与角速度信息
static double vx = 0.0;
static double vy = 0.0; //y方向速度为 0
static double vth = 0.0; //角速度

//处理订阅的速度消息
void carVelCB(const geometry_msgs::Vector3& carVel){
    ROS_INFO("小车线速度：%f,角速度：%f",carVel.x, carVel.y);
    vx = carVel.x;
    vth = carVel.y;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ROS_INFO("订阅小车速度信息，并发布 odom");

    //初始化 ROS 节点
    ros::init(argc,argv,"pub_odom");
    //创建 ROS 句柄
    ros::NodeHandle nh;
    //创建订阅对象
    ros::Subscriber sub = nh.subscribe("/car_vel",10,carVelCB);
    //创建发布对象
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom",10);
    tf::TransformBroadcaster broadcaster;
    //发布逻辑以及实现
    ros::Rate r(10); //10次/s

    //设置统计周期内的起始时刻
    ros::Time last_time = ros::Time::now(); //上次的结束时刻
    ros::Time right_now = ros::Time::now(); //当前时刻

    while(ros::ok()){

        //获取当前时刻
        right_now = ros::Time::now();
        //获取时间差 (当前时刻 - 上次结束时刻)
        double past_time = (right_now - last_time).toSec();
        //计算时间差内小车的位移
        double delta_x = (vx * cos(th) - vy * sin(th)) * past_time;
        double delta_y = (vx * sin(th) + vy * cos(th)) *past_time;
        double delta_th = vth * past_time;
        //坐标累加
        x += delta_x;
        y += delta_y;
        th += delta_th;
        //将结束时刻，赋值为该统计周期的 "当前时刻"
        last_time = right_now;

        //------------------------------------------数据组织与发布--------------------------------------------

        //发布坐标变换
        //先将欧拉角转换成四元数
        geometry_msgs::Quaternion qtn = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped stamped;
        stamped.header.stamp = right_now;//时间戳
        stamped.header.frame_id = "odom";
        stamped.child_frame_id = "base_footprint";
        stamped.transform.translation.x = x;
        stamped.transform.translation.y = y;
        stamped.transform.translation.z = 0.0;
        stamped.transform.rotation = qtn;

        broadcaster.sendTransform(stamped);



        //发布里程计
        nav_msgs::Odometry odom;
        odom.header.stamp = right_now;
        odom.header.frame_id = "odom";

        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation = qtn;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;


        pub.publish(odom);


        //-----------------------------------------------------------------------------------------------------------
        r.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
```

launch 文件:

```xml
<launch>
    <node pkg="rosserial_python" type="serial_node.py" name="serial">
        <param name="port" value="/dev/ttyACM0" />
    </node>
    <node pkg="demo01_odom" type="Hello_Odom" name="odom" />
</launch>
```



