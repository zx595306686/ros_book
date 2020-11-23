### 2.2.2 服务通信自定义srv

srv 文件内的可用数据类型与 msg 文件一致，且定义 srv 实现流程与自定义 msg 实现流程类似:

1. 按照固定格式创建srv文件

2. 编辑配置文件

3. 编译生成中间文件

#### 1.定义srv文件

服务通信中，数据分成两部分，请求与响应，在 srv 文件中请求和响应使用`---`分割，具体实现如下:

功能包下新建 srv 目录，添加 xxx.srv 文件，内容:

```
# 客户端请求时发送的两个数字
int32 num1
int32 num2
---
# 服务器响应发送的数据
int32 sum
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

**CMakeLists.txt**编辑 srv 相关配置

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
add_service_files(
  FILES
  AddInts.srv
)
```

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

注意: 官网没有在 catkin\_package 中配置 message\_runtime,经测试配置也可以

#### 3.编译

编译后的中间文件查看:

C++ 需要调用的中间文件\(.../工作空间/devel/include/包名/xxx.h\)

![](/assets/07vscode_自定义消息的中间文件%28C++%29.PNG)

Python 需要调用的中间文件\(.../工作空间/devel/lib/python3/dist-packages/包名/msg\)

![](/assets/08vscode_自定义消息的中间文件%28Python%29.PNG)

后续调用相关 msg 时，是从这些中间文件调用的

