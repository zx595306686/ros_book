### 2.1.6 话题通信自定义msg调用B\(Python\)

#### 0.vscode配置

为了方便代码提示以及误抛异常，需要先配置 vscode，将前面生成的 python 文件路径配置进 settings.json

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/xxx/yyy工作空间/devel/lib/python3/dist-packages"
    ]
}
```

#### 1.发布方

```py
#! /usr/bin/env python
"""
    发布方:
        循环发送消息

"""
import rospy
from demo02_talker_listener.msg import Person


if __name__ == "__main__":
    #1.初始化 ROS 节点
    rospy.init_node("talker_person_p")
    #2.创建发布者对象
    pub = rospy.Publisher("chatter_person",Person,queue_size=10)
    #3.组织消息
    p = Person()
    p.name = "葫芦瓦"
    p.age = 18
    p.height = 0.75

    #4.编写消息发布逻辑
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(p)  #发布消息
        rate.sleep()  #休眠
        rospy.loginfo("姓名:%s, 年龄:%d, 身高:%.2f",p.name, p.age, p.height)
```

#### 2.订阅方

```py
#! /usr/bin/env python
"""
    订阅方:
        订阅消息

"""
import rospy
from demo02_talker_listener.msg import Person

def doPerson(p):
    rospy.loginfo("接收到的人的信息:%s, %d, %.2f",p.name, p.age, p.height)


if __name__ == "__main__":
    #1.初始化节点
    rospy.init_node("listener_person_p")
    #2.创建订阅者对象
    sub = rospy.Subscriber("chatter_person",Person,doPerson,queue_size=10)
    rospy.spin() #4.循环
```

#### 3.权限设置

终端下进入 scripts 执行:`chmod +x *.py`

#### 4.配置 CMakeLists.txt

```cmake
catkin_install_python(PROGRAMS
  scripts/talker_p.py
  scripts/listener_p.py
  scripts/person_talker.py
  scripts/person_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 5.执行

执行方式同之前。

