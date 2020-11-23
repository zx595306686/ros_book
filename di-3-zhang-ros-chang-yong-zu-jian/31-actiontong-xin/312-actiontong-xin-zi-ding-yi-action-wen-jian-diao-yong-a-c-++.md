### 3.1.2 action通信自定义action文件调用A\(C++\)

#### 0.vscode配置

需要像之前自定义 msg 实现一样配置c\_cpp\_properies.json 文件，如果以前已经配置且没有变更工作空间，可以忽略，如果需要配置，配置方式与之前相同:

```json
{
    "configurations": [
        {
            "browse": {
                "databaseFilename": "",
                "limitSymbolsToIncludedHeaders": true
            },
            "includePath": [
                "/opt/ros/noetic/include/**",
                "/usr/include/**",
                "/xxx/yyy工作空间/devel/include/**" //配置 head 文件的路径 
            ],
            "name": "ROS",
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17"
        }
    ],
    "version": 4
}
```

#### 1.服务端

```cpp
/*
    需求:
        ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整形数据)
        服务器会计算 1 到 N 之间所有整数的和，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，
        为了良好的用户体验，
        需要服务器在循环累加过程中，每累加一次，
        就给客户端响应一次执行进度，使用 action实现

    已经实现:
        定义了 action 消息体

    服务器端实现:
        1.包含头文件
          此处无需包含 ros/ros.h 因为 actionlib 已经包含了 
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建服务器对象
        5.处理请求数据产生响应结果，中间还要连续反馈
        6.spin

*/
#include "actionlib/server/simple_action_server.h"
#include "demo04_action/AddIntsAction.h"

typedef actionlib::SimpleActionServer<demo04_action::AddIntsAction> Server;

// void executeCb(const demo04_action::AddIntsGoal::ConstPtr& g, Server* s){
void executeCb(const demo04_action::AddIntsGoalConstPtr& g, Server* s){
    //1.解析提交的目标值
    int goal = g->add_max_num;
    ROS_INFO("需要计算的目标值:%d --- ",goal);
    //2.循环累加数据，并连续反馈
    ros::Rate r(10);
    int sum = 0;
    for (int i = 1; i <= goal; i++)
    {
        sum += i;
        //响应进度
        demo04_action::AddIntsFeedback fb_obj;
        fb_obj.progress_bar = i / (double)goal;
        s->publishFeedback(fb_obj);
        r.sleep();
    }
    //3.响应最终结果
    //将最终结果封装进对象
    demo04_action::AddIntsResult result_obj;
    result_obj.final_result = sum;
    //响应
    s->setSucceeded(result_obj);
    ROS_INFO("服务器响应的结果:%d",result_obj.final_result);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"action_server");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建服务器对象(类型冗余，可以重命名优化)
    // actionlib::SimpleActionServer<demo04_action::AddIntsAction> 
    /* SimpleActionServer(ros::NodeHandle n, 
                          std::string name, 
                          boost::function<void (const demo04_action::AddIntsGoalConstPtr &)> execute_callback, 
                          bool auto_start)
    */
    Server server(nh,"addInts",boost::bind(&executeCb,_1,&server),false);
    server.start();
    ROS_INFO("action服务已经启动....");
    //     5.处理请求数据产生响应结果，中间还要连续反馈
    //     6.spin
    ros::spin();
    return 0;
}
```

#### 2.客户端

```cpp
/*
    需求:
        ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整形数据)
        服务器会计算 1 到 N 之间所有整数的和，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，
        为了良好的用户体验，
        需要服务器在循环累加过程中，每累加一次，
        就给客户端响应一次执行进度，使用 action实现

    已经实现:
        定义了 action 消息体

    服务器端实现:
        1.包含头文件
          此处无需包含 ros/ros.h 因为 actionlib 已经包含了 
        2.初始化 ROS 节点
        3.创建 ROS 句柄(可以需要，也可以不需要)
        4.创建客户端对象
        5.等待服务启动
        6.组织并发送目标值，回调函数处理连续反馈以及最终响应结果
        7.spin()

*/
#include "actionlib/client/simple_action_client.h"
#include "demo04_action/AddIntsAction.h"

typedef actionlib::SimpleActionClient<demo04_action::AddIntsAction> Client;

void doneCb(const actionlib::SimpleClientGoalState &state, const demo04_action::AddIntsResult::ConstPtr &result){
// void doneCb(const actionlib::SimpleClientGoalState &state, const demo04_action::AddIntsResultConstPtr &result){
    ROS_INFO("任务状态:%d",state.state_);
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("最终结果:%d",result->final_result);
    } else {
        ROS_ERROR("任务失败");
    }

}

void activeCb(){
    ROS_INFO("服务被激活");
}

// void feedbackCb(const demo04_action::AddIntsFeedbackConstPtr &feedback){
void feedbackCb(const demo04_action::AddIntsFeedback::ConstPtr &feedback){
    ROS_INFO("当前进度:%.1f %%",feedback->progress_bar * 100);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"action_client");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建客户端对象(类型冗余，重命名优化)
    // actionlib::SimpleActionClient<demo04_action::AddIntsAction> 
    Client client(nh,"addInts",true);
    //     5.等待服务启动
    client.waitForServer();
    // ros::service::waitForService("addInts"); //不可以使用
    //     6.组织并发送目标值，回调函数处理连续反馈以及最终响应结果
    demo04_action::AddIntsGoal goal_obj;
    goal_obj.add_max_num = 100;

    /*
        void sendGoal
            (const demo04_action::AddIntsGoal &goal, 
            boost::function<void (const actionlib::SimpleClientGoalState &state, const demo04_action::AddIntsResultConstPtr &result)> done_cb, 
            boost::function<void ()> active_cb = actionlib::SimpleActionClient<demo04_action::AddIntsAction>::SimpleActiveCallback(), 
            boost::function<void (const demo04_action::AddIntsFeedbackConstPtr &feedback)> feedback_cb = actionlib::SimpleActionClient<demo04_action::AddIntsAction>::SimpleFeedbackCallback())

    */
    client.sendGoal(goal_obj,&doneCb,&activeCb,&feedbackCb);

    //设置等待时间
    bool flag = client.waitForResult(ros::Duration(5));
    if (flag)
    {
        ROS_INFO("任务顺利结束");

    } else {
        ROS_INFO("任务超时");
        // client.cancelGoal();
    }

    // 7.spin() --- 设置等待时间时，不设置 spin()
    // ros::spin();

    return 0;
}
```

**PS:**

1. 等待服务启动，只可以使用`client.waitForServer();`,之前服务中等待启动的另一种方式`ros::service::waitForService("addInts");`不适用

2. 参数替换: xxxConstPtr 可以和 xxx::ConstPtr 置换

   比如:`demo04_action::AddIntsFeedbackConstPtr`与`demo04_action::AddIntsFeedback::ConstPtr`

#### 3.编译配置文件

```cmake
add_executable(Action_Server src/Action_Server.cpp)
add_executable(Action_Client src/Action_Client.cpp)

...

add_dependencies(Action_Server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Action_Client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

...

target_link_libraries(Action_Server
  ${catkin_LIBRARIES}
)
target_link_libraries(Action_Client
  ${catkin_LIBRARIES}
)
```

#### 4.执行



