## 5.3.3 rqt常用插件:rqt\_console

**简介:**rqt\_console 是 ROS 中用于显示和过滤日志的图形化插件

**准备:**编写 Node 节点输出各个级别的日志信息

```cpp
/*  
    ROS 节点:输出各种级别的日志信息

*/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"log_demo");
    ros::NodeHandle nh;

    ros::Rate r(0.3);
    while (ros::ok())
    {
        ROS_DEBUG("Debug message d");
        ROS_INFO("Info message oooooooooooooo");
        ROS_WARN("Warn message wwwww");
        ROS_ERROR("Erroe message EEEEEEEEEEEEEEEEEEEE");
        ROS_FATAL("Fatal message FFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
        r.sleep();
    }


    return 0;
}
```

**启动:**

可以在 rqt 的 plugins 中添加，或者使用`rqt_console`启动![](/assets/01_rqt_console插件.png)

