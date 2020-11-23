### 4.5.3 编码设置话题名称

话题的名称与节点的命名空间、节点的名称是有一定关系的，话题名称大致可以分为三种类型:

* 全局\(话题参考ROS系统，与节点命名空间平级\)
* 相对\(话题参考的是节点的命名空间，与节点名称平级\)
* 私有\(话题参考节点名称，是节点名称的子级\)

结合编码演示具体关系。

---

#### 1.C++ 实现

演示准备:

1.初始化节点设置一个节点名称

`ros::init(argc,argv,"hello")`

2.设置不同类型的话题

3.启动节点时，传递一个 \_\_ns:= xxx

4.节点启动后，使用 rostopic 查看话题信息

##### 1.1全局名称

**格式:**以`/`开头的名称，和节点名称无关

**比如:**/xxx/yyy/zzz

**示例1:**`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter",1000);`

**结果1:**`/chatter`

**示例2:**`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money",1000);`

**结果2:**`/chatter/money`

##### 1.2相对名称

**格式:**非`/`开头的名称,相对节点名称来确定话题名称

**示例1:**`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);`

**结果1:**`xxx/chatter`

**示例2:**`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money",1000);`

**结果2:**`xxx/chatter/money`

##### 1.3私有名称

**格式:**以`~`开头的名称

**示例1:**

`ros::NodeHandle nh("~");`

`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);`

**结果1:**`/xxx/hello/chatter`

**示例2:**

`ros::NodeHandle nh("~");`

`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money",1000);`

**结果2:**`/xxx/hello/chatter/money`

_PS:当使用_`~`_,而话题名称有时_`/`_开头时，那么话题名称是绝对的_

**示例3:**

`ros::NodeHandle nh("~");`

`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money",1000);`

**结果3:**`/chatter/money`

---

#### 2.Python 实现

演示准备:

1.初始化节点设置一个节点名称

`rospy.init_node("hello")`

2.设置不同类型的话题

3.启动节点时，传递一个 \_\_ns:= xxx

4.节点启动后，使用 rostopic 查看话题信息

##### 2.1全局名称

**格式:**以`/`开头的名称，和节点名称无关

**示例1:**`pub = rospy.Publisher("/chatter",String,queue_size=1000)`

**结果1:**`/chatter`

**示例2:**`pub = rospy.Publisher("/chatter/money",String,queue_size=1000)`

**结果2:**`/chatter/money`

##### 2.2相对名称

**格式:**非`/`开头的名称,相对节点名称来确定话题名称

**示例1:**`pub = rospy.Publisher("chatter",String,queue_size=1000)`

**结果1:**`xxx/chatter`

**示例2:**`pub = rospy.Publisher("chatter/money",String,queue_size=1000)`

**结果2:**`xxx/chatter/money`

##### 2.3私有名称

**格式:**以`~`开头的名称

**示例1:**`pub = rospy.Publisher("~chatter",String,queue_size=1000)`

**结果1:**`/xxx/hello/chatter`

**示例2:**`pub = rospy.Publisher("~chatter/money",String,queue_size=1000)`

**结果2:**`/xxx/hello/chatter/money`

