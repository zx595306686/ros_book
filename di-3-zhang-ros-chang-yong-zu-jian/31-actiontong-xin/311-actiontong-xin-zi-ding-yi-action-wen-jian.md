### 3.1.1 action通信自定义action文件

action、srv、msg 文件内的可用数据类型一致，且三者实现流程类似:

1. 按照固定格式创建action文件

2. 编辑配置文件

3. 编译生成中间文件

#### 1.定义action文件

action 文件组成分为三部分:请求目标值，最终响应结果，连续反馈，三者之间使用`---`分割

示例:首先功能包下新建 action 目录，新增 Xxx.action,内容如下

```
#目标值
int32 add_max_num
---
#最终结果
int32 final_result
---
#连续反馈
float64 progress_bar
```

#### 2.编译配置文件

**package.xml**

```cmake
....
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
....
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

**CMakeLists.txt**

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)
```

```cmake
add_action_files(
  FILES
  AddInts.action
)
```

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
```

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo04_action
 CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs
#  DEPENDS system_lib
)
```

#### 3.编译

编译后会生成一些中间文件:

C++ 需要调用的中间文件\(.../工作空间/devel/include/包名/xxx.h\)

![](/assets/10vscode_自定义action的中间文件%28C++%29.PNG)

Python 需要调用的中间文件\(.../工作空间/devel/lib/python3/dist-packages/包名/msg\)![](/assets/11vscode_自定义action的中间文件%28Python%29.PNG)**PS:**

在 C++ 中间文件中，生成了多个 .h 文件，其中 xxxAction.h 包含了 xxxActionGoal.h、xxxActionResult.h 和xxxActionFeedback.h,后面三者又分别包含了对应的 xxxGoal.h、xxxResult.h 和 xxx.Feedback.h,导包时只需要导入 xxxAction.h 即可

