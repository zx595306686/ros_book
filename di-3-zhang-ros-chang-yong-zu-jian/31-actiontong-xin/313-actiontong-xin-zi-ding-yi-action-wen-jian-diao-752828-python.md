### 3.1.3 action通信自定义action文件调用\(Python\)

#### 0.vscode配置

需要像之前自定义 msg 实现一样配置settings.json 文件，如果以前已经配置且没有变更工作空间，可以忽略，如果需要配置，配置方式与之前相同:

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/xxx/yyy工作空间/devel/lib/python3/dist-packages"
    ]
}
```

#### 1.服务端

```py
#! /usr/bin/env python
"""
    需求:
        ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整形数据)
        服务器会计算 1 到 N 之间所有整数的和，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，
        为了良好的用户体验，
        需要服务器在循环累加过程中，每累加一次，
        就给客户端响应一次执行进度，使用 action实现

    已经实现:
        定义了 action 消息体

    服务器端实现:
        1.导包
        2.初始化 ROS 节点
        3.使用类封装，然后创建对象
            4.创建服务器对象
            5.处理请求数据产生响应结果，中间还要连续反馈
        6.spin

"""
import rospy
import actionlib
from demo04_action.msg import *

class MyActionServer:
    def __init__(self):
        #创建服务对象
        #SimpleActionServer(name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("addInts",AddIntsAction,self.executeCb,False)
        #启动服务
        self.server.start()

    def executeCb(self,goal):
        #1.解析目标值
        goal_num = goal.add_max_num
        rospy.loginfo("提交数据是:%d",goal_num)
        #2.循环累加，连续反馈
        r = rospy.Rate(10)
        sum = 0
        for i in range(1,goal_num + 1):
            sum += i
            fb_obj = AddIntsFeedback()
            fb_obj.progress_bar = i / goal_num
            self.server.publish_feedback(fb_obj)
            rospy.loginfo("当前进度:%.1f %%",fb_obj.progress_bar * 100)
            r.sleep()
        #3.响应最终结果
        result = AddIntsResult()
        result.final_result = sum
        rospy.loginfo("最终结果:%d",sum)
        self.server.set_succeeded(result)
if __name__ == "__main__":
    rospy.init_node("action_server_p")
    server = MyActionServer()
    rospy.spin()
```

#### 2.客户端

```py
#! /usr/bin/env python
"""
    需求:
        ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整形数据)
        服务器会计算 1 到 N 之间所有整数的和，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，
        为了良好的用户体验，
        需要服务器在循环累加过程中，每累加一次，
        就给客户端响应一次执行进度，使用 action实现

    已经实现:
        定义了 action 消息体

    服务器端实现:
        1.导包
        2.初始化 ROS 节点
        3.创建 action Client 对象
        4.等待服务
        5.组织结果对象并发送
        6.编写回调, 激活、连续反馈、最终响应
        7.spin

"""
# 1.导包
import rospy
import actionlib
from demo04_action.msg import *

def activeCb():
    rospy.loginfo("服务被激活....")

def feedbackCb(fb):
    rospy.loginfo("当前进度:%.1f %%",fb.progress_bar * 100)

def doneCb(state,r):
    rospy.loginfo("状态:%d",state)
    rospy.loginfo("最终结果:%d",r.final_result)


if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("action_client_p")
    # 3.创建 action Client 对象
    client = actionlib.SimpleActionClient("addInts",AddIntsAction)
    # 4.等待服务
    client.wait_for_server()
    # 5.组织目标对象并发送
    goal_obj = AddIntsGoal()
    goal_obj.add_max_num = 30
    # 6.编写回调, 激活、连续反馈、最终响应
    client.send_goal(goal_obj,doneCb,activeCb,feedbackCb)

    flag = client.wait_for_result(rospy.Duration(5))
    if flag:
        rospy.loginfo("执行成功!")
    else:
        rospy.loginfo("执行超时!")

    # 7.spin
    # rospy.spin()
```

#### 3.编辑配置文件

先为 Python 文件添加可执行权限:`chmod +x *.py`

```cmake
catkin_install_python(PROGRAMS
  scripts/Action_Server_p.py
  scripts/Action_Client_p.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 4.执行



