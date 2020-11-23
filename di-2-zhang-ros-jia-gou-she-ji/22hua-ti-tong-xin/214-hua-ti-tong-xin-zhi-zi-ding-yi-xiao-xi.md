### 2.1.4 话题通信自定义msg

在 ROS 通信协议中，数据载体是一个较为重要组成部分，ROS 中通过 std\_msgs 封装了一些原生的数据类型,比如:String、Int32、Int64、Char、Bool、Empty.... 但是，这些数据一般只包含一个 data 字段，结构的单一意味着功能上的局限性，当传输一些复杂的数据，比如: 激光雷达的信息... std\_msgs 由于描述性较差而显得力不从心，这种场景下可以使用自定义的消息类型

msgs只是简单的文本文件，每行具有字段类型和字段名称，可以使用的字段类型有：

* int8, int16, int32, int64 \(plus uint\*\)

* float32, float64

* string

* time, duration

* other msg files

* variable-length array\[\] and fixed-length array\[C\]

ROS中还有一种特殊类型：`Header`，标头包含时间戳和ROS中常用的坐标帧信息。会经常看到msg文件的第一行具有`Header标头`。

**需求:**创建自定义消息格式。

**流程:**

1. 按照固定格式创建 msg 文件
2. 编辑配置文件
3. 编译生成可以被 Python 或 C++ 调用的中间文件

#### 1.定义msg文件

功能包下新建 msg 目录，添加文件 Person.msg

```
string name
uint16 age
float64 height
```

#### 2.编辑配置文件

**package.xml**中添加编译依赖与执行依赖

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <!-- 
  exce_depend 以前对应的是 run_depend 现在非法
  -->
```

**CMakeLists.txt**编辑 msg 相关配置

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
# 需要加入 message_generation,必须有 std_msgs
```

```
## 配置 msg 源文件
add_message_files(
  FILES
  Person.msg
)
```

```
# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

```
#执行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

#### 3.编译

**编译后的中间文件查看:**

C++ 需要调用的中间文件\(.../工作空间/devel/include/包名/xxx.h\)

![](/assets/05vscode_自定义消息的中间文件%28C++%29.PNG)

Python 需要调用的中间文件\(.../工作空间/devel/lib/python3/dist-packages/包名/msg\)

![](/assets/06vscode_自定义消息的中间文件%28Python%29.PNG)

后续调用相关 msg 时，是从这些中间文件调用的

