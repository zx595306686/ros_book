### 3.2.2 动态配置参数服务端A\(C++\)

#### 0.vscode配置

需要像之前自定义 msg 实现一样配置settings.json 文件，如果以前已经配置且没有变更工作空间，可以忽略，如果需要配置，配置方式与之前相同:

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

#### 1.服务器代码实现

```cpp
/*  
    动态参数服务端: 参数被修改时直接打印
    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.创建服务器对象
        4.创建回调对象(使用回调函数，打印修改后的参数)
        5.服务器对象调用回调对象
        6.spin()

*/

// 1.包含头文件
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h" //动态参数服务
#include "demo06_dynamic_reconfigure/drConfig.h" //配置头文件

void cb(demo06_dynamic_reconfigure::drConfig& config, uint32_t level){
    ROS_INFO("动态参数服务器解析数据:%d,%.2f,%d,%s,%d",
            config.int_param,
            config.double_param,
            config.bool_param,
            config.string_param.c_str(),
            config.list_param
    );
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    // 2.初始化 ros 节点
    ros::init(argc,argv,"dr");
    // 3.创建服务器对象
    dynamic_reconfigure::Server<demo06_dynamic_reconfigure::drConfig> server;
    // 4.创建回调对象(使用回调函数，打印修改后的参数)
    dynamic_reconfigure::Server<demo06_dynamic_reconfigure::drConfig>::CallbackType cbType;
    cbType = boost::bind(&cb,_1,_2);
    // 5.服务器对象调用回调对象
    server.setCallback(cbType);
    // 6.spin()
    ros::spin();
    return 0;
}
```

#### 2.编译配置文件

```cmake
add_executable(DR_Server src/DR_Server.cpp)

add_dependencies(DR_Server ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(DR_Server
  ${catkin_LIBRARIES}
)
```

#### 3.执行

先启动`roscore`

启动服务端:`rosrun 功能包 xxxx`

启动客户端:`rosrun rqt_gui rqt_gui -s rqt_reconfigure`或`rosrun rqt_reconfigure rqt_reconfigure`

![](/assets/14vscode_自定义cfg_执行.PNG)

修改图形化界面数据，服务器将输出修改后的结果

  


