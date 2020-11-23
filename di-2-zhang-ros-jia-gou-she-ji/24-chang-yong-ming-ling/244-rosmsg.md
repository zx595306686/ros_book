### 2.4.4 rosmsg

rosmsg是用于显示有关 ROS消息类型的 信息的命令行工具。

**rosmsg 演示**

```
rosmsg show	显示消息描述
rosmsg info	显示消息信息
rosmsg list	列出所有消息
rosmsg md5	显示 md5 加密后的消息
rosmsg package	显示某个功能包下的所有消息
rosmsg packages	列出包含消息的功能包
```

* rosmsg list

  会列出当前 ROS 中的所有 msg

* rosmsg packages

  列出包含消息的所有包

* rosmsg package

  列出某个包下的所有msg

  ```
  //rosmsg package 包名 
  rosmsg package turtlesim 
  ```

* rosmsg show

  显示消息描述

  ```
  //rosmsg show 消息名称
  rosmsg show turtlesim/Pose
  结果:
  float32 x
  float32 y
  float32 theta
  float32 linear_velocity
  float32 angular_velocity
  ```

* rosmsg info

  作用与 rosmsg show 一样

* rosmsg md5 \(资料:[http://wiki.ros.org/ROS/Technical%20Overview\#Message\_serialization\_and\_msg\_MD5\_sums](http://wiki.ros.org/ROS/Technical%20Overview#Message_serialization_and_msg_MD5_sums)\)

  一种校验算法，保证数据传输的一致性



