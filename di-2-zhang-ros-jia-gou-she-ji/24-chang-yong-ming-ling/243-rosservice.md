### 2.4.3 rosservice

rosservice包含用于列出和查询ROS[Services](http://wiki.ros.org/Services)的rosservice命令行工具。

调用部分服务时，如果对相关工作空间没有配置 path，需要进入工作空间调用 source ./devel/setup.bash

```
rosservice args 打印服务参数
rosservice call	使用提供的参数调用服务
rosservice find	按照服务类型查找服务
rosservice info	打印有关服务的信息
rosservice list	列出所有活动的服务
rosservice type	打印服务类型
rosservice uri	打印服务的 ROSRPC uri
```

* rosservice list

  列出所有活动的 service

  ```
  ~ rosservice list
  /clear
  /kill
  /listener/get_loggers
  /listener/set_logger_level
  /reset
  /rosout/get_loggers
  /rosout/set_logger_level
  /rostopic_4985_1578723066421/get_loggers
  /rostopic_4985_1578723066421/set_logger_level
  /rostopic_5582_1578724343069/get_loggers
  /rostopic_5582_1578724343069/set_logger_level
  /spawn
  /turtle1/set_pen
  /turtle1/teleport_absolute
  /turtle1/teleport_relative
  /turtlesim/get_loggers
  /turtlesim/set_logger_level
  ```

* rosservice args

  打印服务参数

  ```
  rosservice args /spawn
  x y theta name
  ```

* rosservice call

  调用服务

  为小乌龟的案例生成一只新的乌龟

  ```
  rosservice call /spawn "x: 1.0
  y: 2.0
  theta: 0.0
  name: 'xxx'"
  name: "xxx"

  //生成一只叫 xxx 的乌龟
  ```

* rosservice find

  根据消息类型获取话题

* rosservice info

  获取服务话题详情

* rosservice type

  获取消息类型

* rosservice uri

  获取服务器 uri



