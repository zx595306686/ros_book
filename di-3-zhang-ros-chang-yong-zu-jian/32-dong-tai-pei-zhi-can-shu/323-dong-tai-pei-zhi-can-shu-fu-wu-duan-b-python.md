### 3.2.3 动态配置参数服务端B\(Python\)

#### 0.vscode配置

需要像之前自定义 msg 实现一样配置settings.json 文件，如果以前已经配置且没有变更工作空间，可以忽略，如果需要配置，配置方式与之前相同:

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/xxx/yyy工作空间/devel/lib/python3/dist-packages"
    ]
}
```

#### 1.服务器代码实现

```py
#! /usr/bin/env python
""" 
    动态参数服务端: 参数被修改时直接打印
    实现流程:
        1.导包
        2.初始化 ros 节点
        3.创建服务对象
        4.回调函数处理
        5.spin
"""



# 1.导包
import rospy
from dynamic_reconfigure.server import Server
from demo06_dynamic_reconfigure.cfg import drConfig

def cb(config,level):
    rospy.loginfo("python 动态参数服务解析:%d,%.2f,%d,%s,%d",
                config.int_param,
                config.double_param,
                config.bool_param,
                config.string_param,
                config.list_param
                )
    return config

if __name__ == "__main__":

    # 2.初始化 ros 节点
    rospy.init_node("dr_p")

    # # 3.创建服务对象
    # # 4.回调函数处理
    server = Server(drConfig,cb)

    # # 5.spin
    rospy.spin()
    pass
```

#### 2.编辑配置文件

先为 Python 文件添加可执行权限:`chmod +x *.py`

```cmake
catkin_install_python(PROGRAMS
  scripts/DR_Server_p.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 3.执行

先启动`roscore`

启动服务端:`rosrun 功能包 xxxx.py`

启动客户端:`rosrun rqt_gui rqt_gui -s rqt_reconfigure`或`rosrun rqt_reconfigure rqt_reconfigure`

![](/assets/14vscode_自定义cfg_执行.PNG)

修改图形化界面数据，服务器将输出修改后的结果

