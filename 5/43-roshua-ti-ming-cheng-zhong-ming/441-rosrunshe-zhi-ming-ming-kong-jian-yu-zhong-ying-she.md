### 4.5.1 rosrun设置话题重映射

**rosrun名称重映射语法: rorun 包名 节点名 话题名:=新话题名称**

实现teleop\_twist\_keyboard与乌龟显示节点通信方案由两种：

##### 1.方案1

将 teleop\_twist\_keyboard 节点的话题设置为`/turtle1/cmd_vel`

启动键盘控制节点:`rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel`

启动乌龟显示节点: `rosrun turtlesim turtlesim_node`

二者可以实现正常通信

##### 2.方案2

将乌龟显示节点的话题设置为 `/cmd_vel`

启动键盘控制节点:`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

启动乌龟显示节点: `rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel`

二者可以实现正常通信

