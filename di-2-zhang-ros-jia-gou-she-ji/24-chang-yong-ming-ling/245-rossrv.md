### 2.4.5 rossrv

rossrv是用于显示有关ROS服务类型的信息的命令行工具，与 rosmsg 使用语法高度雷同。

```
rossrv show	显示服务消息详情
rossrv info	显示服务消息相关信息
rossrv list	列出所有服务信息
rossrv md5	显示 md5 加密后的服务消息
rossrv package	显示某个包下所有服务消息
rossrv packages	显示包含服务消息的所有包
```

* rossrv list

  会列出当前 ROS 中的所有 srv 消息

* rossrv packages

  列出包含服务消息的所有包

* rossrv package

  列出某个包下的所有msg

  ```
  //rossrv package 包名 
  rossrv package turtlesim 
  ```

* rossrv show

  显示消息描述

  ```
  //rossrv show 消息名称
  rossrv show turtlesim/Spawn
  结果:
  float32 x
  float32 y
  float32 theta
  string name
  ---
  string name
  ```

* rossrv info

  作用与 rossrv show 一致

* rossrv md5

  对 service 数据使用 md5 校验\(加密\)



