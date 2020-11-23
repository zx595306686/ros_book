### 2.4.2 rostopic

**rostopic**包含rostopic命令行工具，用于显示有关ROS 主题的调试信息，包括发布者，订阅者，发布频率和ROS消息。它还包含一个实验性Python库，用于动态获取有关主题的信息并与之交互。

```
rostopic bw     显示主题使用的带宽
rostopic delay  显示带有 header 的主题延迟
rostopic echo   打印消息到屏幕
rostopic find   根据类型查找主题
rostopic hz     显示主题的发布频率
rostopic info   显示主题相关信息
rostopic list   显示所有活动状态下的主题
rostopic pub    将数据发布到主题
rostopic type   打印主题类型
```

* **rostopic list**\(-v\)

  直接调用即可，控制台将打印当前运行状态下的主题名称

  rostopic list -v : 获取话题详情\(比如列出：发布者和订阅者个数...\)

* **rostopic pub**

  可以直接调用命令向订阅者发布消息

  为roboware 自动生成的 发布/订阅 模型案例中的 订阅者 发布一条字符串

  ```
  rostopic pub /主题名称 消息类型 消息内容
  rostopic pub /chatter std_msgs gagaxixi
  ```

  为 小乌龟案例的 订阅者 发布一条运动信息

  ```
  rostopic pub /turtle1/cmd_vel geometry_msgs/Twist
   "linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 2.0"
  //只发布一次运动信息

  rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist
   "linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 2.0"
  // 以 10HZ 的频率循环发送运动信息
  ```

* **rostpic echo**

  获取指定话题当前发布的消息

* **rostopic info**

  获取当前话题的小关信息

  消息类型

  发布者信息

  订阅者信息

* **rostopic type**

  列出话题的消息类型

* **rostopic find 消息类型**

  根据消息类型查找话题

* **rostopic delay**

  列出消息头信息

* **rostopic hz**

  列出消息发布频率

* **rostopic bw**

  列出消息发布带宽



