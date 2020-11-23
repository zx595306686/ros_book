### 3.1.4 action通信实操

**需求描述: **让小乌龟到达指定目标点，并且在运行过程中，时时返回当前位姿信息。

**结果演示:**

**实现分析:**

1. 需要启动乌龟显示节点。
2. 需求是带连续反馈的请求，主体需要使用 action 实现。
3. 客户端请求需要发送目标点，并接收反馈。
4. 服务端需要接收请求坐标，然后控制乌龟运动，并连续反馈乌龟位姿\(先订阅再反馈\)。
5. 必须了解乌龟控制与位姿使用的的话题和消息。

**实现流程:**

1. 通过 ros 命令来获取乌龟运动控制、乌龟位姿相关的话题和消息。
2. 定义action文件来设置请求数据、响应结果与连续反馈数据。
3. 定义action客户端向服务器发送请求，并处理响应。
4. 定义action服务端解析目标坐标，控制乌龟向坐标点运动，反馈位姿信息，响应最终执行结果。
5. 执行。

#### 1.准备

**控制乌龟运动，那么首先需要了解:乌龟节点 turtlesim\_node 订阅的话题以及使用的消息类型**

**获取话题:**/turtle1/cmd\_vel

```
rostopic list
```

**获取消息类型:**geometry\_msgs/Twist

```
rostopic type /turtle1/cmd_vel
```

**获取消息格式:**

```
rosmsg info geometry_msgs/Twist
```

响应结果:

```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

**订阅乌龟在窗体中坐标\(位姿\)，需要先了解乌龟节点 turtlesim\_node 发布的话题以及消息类型**

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

响应结果:

```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

#### 2.定义action文件

action 文件需要设置目标点的坐标点、最终到达为止的坐标点、连续反馈的乌龟的位姿。

```

```

#### 3-1.action 实现方案A\(C++\)

**Server 端:**

```cpp
/*
    通过 action 控制乌龟运动到指定的目的地，移动过程中连续接收乌龟反馈

    分析:
        1.使用action
        2.需要订阅乌龟的位姿
        3.需要发布乌龟运动的速度信息

    实现流程(阶段1:实现action通信):
        1.包含头文件
        2.初始化 ros 节点
        3.创建 ros 句柄
        4.创建 action server 对象
        5.处理请求
        6.spin
    实现流程(阶段2:实现位姿订阅与速度发布)
        实现逻辑:
        1.在回调函数中，需要解析目标值

        2.根据目标值计算生成速度指令并发布
        3.并且需要时时反馈乌龟位姿

        4.到达目标点后生成最终响应结果

*/
//1.包含头文件
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "demo02_test_action/turtle_moveAction.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

typedef actionlib::SimpleActionServer<demo02_test_action::turtle_moveAction> Server;

//定义位姿相关的结构体
struct TurtlePose {
    double x;
    double y;
    double z;
}final_pose,current_pose,goal_pose;
ros::Publisher pub;


//处理请求
void doReq(const demo02_test_action::turtle_moveGoalConstPtr& goal, Server* s){
    ROS_INFO("目标值:x=%.2f, y=%.2f, theta=%.2f",goal->turtle_goal_x,goal->turtle_goal_y,goal->turtle_goal_z);
    goal_pose.x = goal->turtle_goal_x;
    goal_pose.y = goal->turtle_goal_y;
    goal_pose.z = goal->turtle_goal_z;

    demo02_test_action::turtle_moveFeedback fb;
    geometry_msgs::Twist vec_msg;
    demo02_test_action::turtle_moveResult result;

    ros::Rate r(10);
    while(true)
    {   
        //已有目标坐标信息:goal_pose, 和当前坐标信息: current_pose
        //根据二者计算出角度与速度
        // 线速度 = ((目标x - 当前x)平方 + (目标y - 当前y)平方)然后再开方 再 * 速度系数
        double jianju = sqrt(pow(goal_pose.x - current_pose.x,2) + pow(goal_pose.y - current_pose.y,2));
        vec_msg.linear.x = 0.5 * jianju;
        // 角速度 = (atan2(目标y - 当前y, 目标x - 当前x) - 当前角度) * 系数
        vec_msg.angular.z = 2 * (atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x) - current_pose.z);
        //将计算后的结果通过 pub 发送
        pub.publish(vec_msg);

        //将当前坐标信息赋值给 fb
        fb.turtle_current_x = current_pose.x;
        fb.turtle_current_y = current_pose.y;
        fb.turtle_current_z = current_pose.z;
        s->publishFeedback(fb);

        //还要设置循环结束的条件(比如:距离目标点 0.1m 之内时，即中止)
        if (jianju < 0.1)
        {
            break;

        }


        r.sleep();

    }


    result.turtle_final_x = current_pose.x;
    result.turtle_final_y = current_pose.y;
    result.turtle_final_z = current_pose.z;

    s->setSucceeded(result);

}

//订阅位姿的回调函数，将订阅的坐标信息保存进结构体
void doPose(const turtlesim::Pose::ConstPtr& pose){
    current_pose.x = pose->x;
    current_pose.y = pose->y;
    current_pose.z = pose->theta;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"move_server");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 action server 对象

    //创建 pose 的订阅节点
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1000,doPose);

    //创建 cmd_vel 的发布节点
    pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);

    //boost::function<void (const demo02_test_action::turtle_moveGoalConstPtr &)> execute_callback
    Server server(nh,"turtle_move",boost::bind(&doReq,_1,&server), false);
    //启动服务
    server.start();
    ROS_INFO("服务启动成功！！！！");
    //     5.处理请求
    //     6.spin
    ros::spin();
    return 0;
}
```

**Client 端:**

```cpp
/*  
    通过 action 控制乌龟运动到指定的目的地，移动过程中连续接收乌龟反馈

    分析:
        1.使用action
        2.需要订阅乌龟的位姿
        3.需要发布乌龟运动的速度信息

    实现流程(阶段1:实现action通信):
        1.包含头文件
        2.初始化 ros 节点
        3.创建 ros 句柄
        4.创建 action client 对象
        5.发送请求
        6.spin


*/

//1.包含头文件
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "demo02_test_action/turtle_moveAction.h"

typedef actionlib::SimpleActionClient<demo02_test_action::turtle_moveAction> Client;


//服务被激活函数
void activeCb(){
    ROS_INFO("服务已经连接成功");
}

//连续反馈函数
void fbCb(const demo02_test_action::turtle_moveFeedbackConstPtr &feedback){
    ROS_INFO("连续反馈:x=%.2f,y=%.2f,z=%.2f;",
            feedback->turtle_current_x,
            feedback->turtle_current_y,
            feedback->turtle_current_z
            );
}

//最终响应结果
void  doneCb(const actionlib::SimpleClientGoalState &state, 
            const demo02_test_action::turtle_moveResultConstPtr &result){
    ROS_INFO("响应状态:%d",state.state_);
    ROS_INFO("最终-------------反馈:x=%.2f,y=%.2f,z=%.2f;",
            result->turtle_final_x,
            result->turtle_final_y,
            result->turtle_final_z
            );
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"move_client");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 action client 对象
    Client client(nh,"turtle_move",true);
    client.waitForServer();
    // 5.发送请求
    //void sendGoal(const demo02_test_action::turtle_moveGoal &goal, 
    //              boost::function<void (const actionlib::SimpleClientGoalState &state, const demo02_test_action::turtle_moveResultConstPtr &result)> done_cb, 
    //              boost::function<void ()> active_cb, 
    //              boost::function<void (const demo02_test_action::turtle_moveFeedbackConstPtr &feedback)> feedback_cb)
    demo02_test_action::turtle_moveGoal goal;
    nh.getParam("x",goal.turtle_goal_x);
    nh.getParam("y",goal.turtle_goal_y);
    nh.getParam("z",goal.turtle_goal_z);

    client.sendGoal(goal,&doneCb, &activeCb, &fbCb);
    // 6.spin
    ros::spin();
    return 0;
}
```

#### 3-2.action 实现方案B\(Python\)

**Server 端:**

```py
#! /usr/bin/env python
""" 
    通过 action 控制乌龟运动到指定的目的地，移动过程中连续接收乌龟反馈

    分析:
        1.使用action
        2.需要订阅乌龟的位姿
        3.需要发布乌龟运动的速度信息

    实现流程(阶段1:实现action通信):
        1.包含头文件
        2.初始化 ros 节点
        3.创建 action server 对象
        4.处理请求
        5.spin
    实现流程(阶段2:实现位姿订阅与速度发布)


"""
import rospy
from demo02_test_action.msg import turtle_moveAction, turtle_moveGoal, turtle_moveResult, turtle_moveFeedback
from actionlib import SimpleActionServer
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
#单独创建一个类来处理请求
class MyServer:
    def __init__(self,topic):
        self.pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1000)
        self.sub = rospy.Subscriber("/turtle1/pose",Pose,self.doPose,queue_size=1000)
        self.server = SimpleActionServer(topic,turtle_moveAction,self.exeCb,False)
        self.server.start()
        rospy.loginfo("服务已经启动....")

    def doPose(self,pose):
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_z = pose.theta

    def exeCb(self,goal):
        #解析目标数据
        goal_x = goal.turtle_goal_x
        goal_y = goal.turtle_goal_y
        goal_z = goal.turtle_goal_z
        rospy.loginfo("目标点:%.2f,%.2f,%.2f",goal_x,goal_y,goal_z)

        #根据目标数据与当前坐标计算速度与角速度，并发布
        #连续回传当前坐标信息
        rate = rospy.Rate(10)
        while True:
            fb = turtle_moveFeedback()
            msg = Twist()
            try:
                # 订阅到的乌龟的位姿
                fb.turtle_current_x = self.current_x
                fb.turtle_current_y = self.current_y
                fb.turtle_current_z = self.current_z
                self.server.publish_feedback(fb)
                # 计算并发布速度信息
                jianju = math.sqrt(math.pow(goal_x - self.current_x,2) + math.pow(goal_y - self.current_y,2))
                msg.linear.x = 0.5 * jianju
                msg.linear.y = 0
                msg.linear.z = 0

                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = 2 * (math.atan2(goal_y - self.current_y, goal_x - self.current_x) - self.current_z);

                self.pub.publish(msg)
                if jianju < 0.1:
                    break
            except Exception as e:
                rospy.logwarn("还没有订阅到乌龟信息，请稍候...")
            rate.sleep()




        #响应最终信息
        final_turtle = turtle_moveResult() 
        final_turtle.turtle_final_x = self.current_x
        final_turtle.turtle_final_y = self.current_y
        final_turtle.turtle_final_z = self.current_z

        self.server.set_succeeded(final_turtle)

if __name__ == "__main__":
    rospy.init_node("move_server_p")
    server = MyServer("turtle_move")
    rospy.spin()
```

**Client 端:**

```py
#! /usr/bin/env python
"""  
    通过 action 控制乌龟运动到指定的目的地，移动过程中连续接收乌龟反馈

    分析:
        1.使用action
        2.需要订阅乌龟的位姿
        3.需要发布乌龟运动的速度信息

    实现流程(阶段1:实现action通信):
        1.包含头文件
        2.初始化 ros 节点
        3.创建 action client 对象
        4.发送请求
        5.spin

"""
import rospy
from demo02_test_action.msg import turtle_moveAction, turtle_moveFeedback, turtle_moveGoal, turtle_moveResult
from actionlib import SimpleActionClient
from actionlib import GoalStatus

def activeCb():
    rospy.loginfo("服务已经激活....")

def fbCb(fb):
    rospy.loginfo("连续反馈的坐标信息:x=%.2f,y=%.2f,z=%.2f",
                    fb.turtle_current_x,
                    fb.turtle_current_y,
                    fb.turtle_current_z
                )

def doneCb(state,result):
    rospy.loginfo("成功了吗?%d",(state == GoalStatus.SUCCEEDED))
    rospy.loginfo("最终终点坐标:x=%.2f,y=%.2f,z=%.2f",
                    result.turtle_final_x,
                    result.turtle_final_y,
                    result.turtle_final_z
                )

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("move_client_p")
    # 3.创建 action client 对象
    client = SimpleActionClient("turtle_move",turtle_moveAction)
    # 5.发送请求
    client.wait_for_server()
    goal = turtle_moveGoal()
    goal.turtle_goal_x = rospy.get_param("x")
    goal.turtle_goal_y = rospy.get_param("y")
    goal.turtle_goal_z = rospy.get_param("z")
    client.send_goal(goal,doneCb,activeCb,fbCb)
    # 6.spin
    rospy.spin()
```

#### 4-1.执行方案A:C++

```xml
<launch>
    <!-- 启动乌龟节点 -->
    <param name="x" value="1.0" />
    <param name="y" value="2.0" />
    <param name="z" value="3.14" />

    <node pkg="turtlesim" type="turtlesim_node" name="turtle" output="screen" />
    <node pkg="demo02_test_action" type="turtle_move_server" name="turtle_server" output="screen" />
    <node pkg="demo02_test_action" type="turtle_move_client" name="turtle_client" output="screen" />
</launch>
```

#### 4-2.执行方案B:C++

```xml
<launch>
    <!-- 启动乌龟节点 -->
    <param name="x" value="1.0" />
    <param name="y" value="2.0" />
    <param name="z" value="3.14" />

    <node pkg="turtlesim" type="turtlesim_node" name="turtle" output="screen" />
    <!-- <node pkg="demo02_test_action" type="turtle_move_server" name="turtle_server" output="screen" /> -->
    <node pkg="demo02_test_action" type="turtle_move_server_p.py" name="turtle_server_p" output="screen" />
    <!-- <node pkg="demo02_test_action" type="turtle_move_client" name="turtle_client" output="screen" /> -->
    <node pkg="demo02_test_action" type="turtle_move_client_p.py" name="turtle_client_p" output="screen" />
</launch>
```



