### 5.2.2 rosbag使用\_编码

命令实现不够灵活，可以使用编码的方式，增强录制与回放的灵活性

---

方案A:C++实现

#### 1.写 bag

```cpp
/* 
    rosbag 写数据
*/
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"bag_w");
    ros::NodeHandle nh;

    //创建 bag 对象
    rosbag::Bag bag;

    //打开文件流
    //此相对路径相对于当前工作空间而言的
    bag.open("../bag/test_bag_write.bag",rosbag::BagMode::Write);
    //写数据
    std_msgs::String str;
    str.data = "hello";

    bag.write("chatter",ros::Time::now(),str);
    bag.write("chatter",ros::Time::now(),str);
    bag.write("chatter",ros::Time::now(),str);

    //关闭流
    bag.close();

    return 0;
}
```

#### 2.读bag

```cpp
/*  
    读取 bag 文件：

*/
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

int main(int argc, char *argv[])
{

    setlocale(LC_ALL,"");

    ros::init(argc,argv,"r_bag");
    ros::NodeHandle nh;

    //创建 bag 对象
    rosbag::Bag bag;
    //打开 bag 文件
    bag.open("../bag/test_bag_write.bag",rosbag::BagMode::Read);
    //读数据
    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();
        if(p != nullptr){
            ROS_INFO("读取的数据:%s",p->data.c_str());
        }
    }

    //关闭文件流
    bag.close();
    return 0;
}
```

---

方案B:Python实现

#### 1.写 bag

```py
#! /usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String

if __name__ == "__main__":
    #初始化节点 
    rospy.init_node("w_bag_p")

    # 创建 rosbag 对象
    bag = rosbag.Bag("../bag/test_write_p.bag",'w')

    # 写数据
    s = String()
    s.data= "hahahaha"

    bag.write("chatter",s)
    bag.write("chatter",s)
    bag.write("chatter",s)
    # 关闭流
    bag.close()
```

#### 2.读bag

```py
#! /usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String

if __name__ == "__main__":
    #初始化节点 
    rospy.init_node("w_bag_p")

    # 创建 rosbag 对象
    bag = rosbag.Bag("../bag/test_write_p.bag",'r')
    # 读数据
    bagMessage = bag.read_messages("chatter")
    for topic,msg,t in bagMessage:
        rospy.loginfo("%s,%s,%s",topic,msg,t)
    # 关闭流
    bag.close()
```



