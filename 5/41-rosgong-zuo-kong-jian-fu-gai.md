## 4.3 ROS工作空间覆盖

所谓工作空间覆盖，是指不同工作空间中，存在重名的功能包的情形。

> ROS 开发中，会自定义工作空间且自定义工作空间可以同时存在多个，可能会出现一种情况: 虽然特定工作空间内的功能包不能重名，但是自定义工作空间的功能包与内置的功能包可以重名或者不同的自定义的工作空间中也可以出现重名的功能包，那么调用该名称功能包时，会调用哪一个呢？比如：自定义工作空间A存在功能包 turtlesim，自定义工作空间B也存在功能包 turtlesim，当然系统内置空间也存在turtlesim，如果调用turtlesim包，会调用哪个工作空间中的呢？

---

#### **实现**

0.新建工作空间A与工作空间B，两个工作空间中都创建功能包: turtlesim。

1.在 ~/.bashrc 文件下**追加**当前工作空间的 bash 格式如下:

```
source /home/用户/路径/工作空间A/devel/setup.bash
source /home/用户/路径/工作空间B/devel/setup.bash
```

2.新开命令行:`source .bashrc`加载环境变量

3.查看ROS环境环境变量`echo $ROS_PACKAGE_PATH`

结果:自定义工作空间B:自定义空间A:系统内置空间

4.调用命令:`roscd turtlesim`会进入自定义工作空间B

#### **原因**

ROS 会解析 .bashrc 文件，并生成 ROS\_PACKAGE\_PATH ROS包路径，该变量中按照 .bashrc 中配置设置工作空间优先级，在设置时需要遵循一定的原则:ROS\_PACKAGE\_PATH 中的值，和 .bashrc 的配置顺序相反---&gt;后配置的优先级更高，如果更改自定义空间A与自定义空间B的source顺序，那么调用时，将进入工作空间A。

#### **结论**

功能包重名时，会按照 ROS\_PACKAGE\_PATH 查找，配置在前的会优先执行。

#### **隐患**

存在安全隐患，比如当前工作空间B优先级更高，意味着当程序调用 turtlesim 时，不会调用工作空间A也不会调用系统内置的 turtlesim，如果工作空间A在实现时有其他功能包依赖于自身的 turtlesim，而按照ROS工作空间覆盖的涉及原则，那么实际执行时将会调用工作空间B的turtlesim，从而导致执行异常，出现安全隐患。

---

BUG 说明:

> 当在 .bashrc 文件中 source 多个工作空间后，可能出现的情况，在 ROS PACKAGE PATH 中只包含两个工作空间，可以删除自定义工作空间的 build 与 devel 目录，重新 catkin\_make，然后重新载入 .bashrc 文件，问题解决。_ _



