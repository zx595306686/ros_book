### 5.2.3 rosbag实操A\(C++\)

**需求:**建档的发布订阅模型,发布消息编号,如果编号在\[50-80\]之间,那么写入bag文件,然后读取 bag 文件，对数据进一步筛选，只取出 大于等于50 且小于 60 的消息，然后发布

---

方案A:C++实现

#### 1.写bag

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>
#include "rosbag/bag.h"

int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能
    //创建 bag 对象
    rosbag::Bag bag;
    //打开 bag 文件流
    bag.open("../bag/chatter.bag",rosbag::BagMode::Write);

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std_msgs::String msg;
    // msg.data = "你好啊！！！";
    std::string msg_front = "Hello 你好！"; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(10);

    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << count;
        msg.data = ss.str();
        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());
        if (count >= 50 && count <= 80)
        {
            bag.write("chatter",ros::Time::now(),msg);
        }


        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //暂无应用
        ros::spinOnce();
    }

    bag.close();

    return 0;
}
```

#### 2.读bag

```cpp
/*  
    读取 bag 文件中的 /turtle1/cmd_vel 数据，然后筛选非 0 数据再发布
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include <string>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"pub_cmd");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);
    //读取 bag 文件
    rosbag::Bag bag;
    //筛选 bag 数据，并发布
    bag.open("../bag/chatter.bag",rosbag::BagMode::Read);
    int count;
    ros::Rate r(3);
    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {

        std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();

        std::string str = p->data;

        int x = std::stoi(str);

        if (x >= 50 && x < 60)
        {
            pub.publish(*p);
        }

        r.sleep();

        ros::spinOnce();

    }
    //关闭 流
    bag.close();
    return 0;
}
```

---

方案B:Python实现

#### 1.写bag

```py
#1.导包 
import rospy
from std_msgs.msg import String
from rosbag import Bag

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("talker_p")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("chatter",String,queue_size=10)

    #创建 Bag 对象
    bag = Bag("../bag/chatter_p.bag",'w')

    #4.组织被发布的数据，并编写逻辑发布数据
    msg = String()  #创建 msg 对象
    msg_front = "hello 你好"
    count = 0  #计数器 
    # 设置循环频率
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        #拼接字符串
        msg.data = msg_front + str(count)

        #加入判断
        if count >= 10 and count <= 20:
            bag.write("chatter",msg)

        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("写出的数据:%s",msg.data)
        count += 1

    bag.close()
```

#### 2.读bag

```py
#1.导包 
import rospy
from std_msgs.msg import String
import rosbag

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("talker_p")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("chatter",String,queue_size=10)
    bag = rosbag.Bag("../bag/chatter.bag",'r')
    #4.组织被发布的数据，并编写逻辑发布数据
    rate = rospy.Rate(1)
    bagMessage = bag.read_messages("chatter")
    for topic,msg,t in bagMessage:
        str = msg.data
        x = int(str)
        if x >= 50 and x <= 55:
            pub.publish(msg)
            rate.sleep()
    
    bag.close()
```



