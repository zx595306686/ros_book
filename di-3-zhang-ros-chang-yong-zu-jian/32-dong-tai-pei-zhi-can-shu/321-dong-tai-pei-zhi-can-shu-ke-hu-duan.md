### 3.2.1 动态配置参数客户端

#### 1.新建功能包，注意添加依赖包: dynamic\_reconfigure

**package.xml 如下: **

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>dynamic_reconfigure</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>dynamic_reconfigure</exec_depend>
```

可以创建功能包时直接声明`roscpp rospy std_msgs dynamic_reconfigure`, 也可以在功能包创建完毕后，手动修改 package.xml

#### 2.新建 cfg 文件夹，添加 xxx.cfg 文件\(并添加可执行权限\)

cfg 文件其实就是一个 python 文件,用于生成参数修改的客户端\(GUI\)。

```py
#! /usr/bin/env python

"""
    生成动态参数 int,double,bool,string,列表
    实现流程:
        1.导包
        2.创建生成器
        3.向生成器添加若干参数
        4.生成中间文件并退出
"""

# 1.导包
from dynamic_reconfigure.parameter_generator_catkin import *
PACKAGE = "demo06_dynamic_reconfigure"

# 2.创建生成器
gen = ParameterGenerator()

# 3.向生成器添加若干参数
#add(name, paramtype, level, description, default=None, min=None, max=None, edit_method="")
gen.add("int_param",int_t,0,"整型参数",50,0,100)
gen.add("double_param",double_t,0,"浮点参数",1.57,0,3.14)
gen.add("string_param",str_t,0,"字符串参数","hello world ")
gen.add("bool_param",bool_t,0,"bool参数",True)

many_enum = gen.enum([gen.const("small",int_t,0,"a small size"),
                    gen.const("mediun",int_t,1,"a medium size"),
                    gen.const("big",int_t,2,"a big size")
                    ],"a car size set")

gen.add("list_param",int_t,0,"列表参数",0,0,2, edit_method=many_enum)

# 4.生成中间文件并退出
exit(gen.generate(PACKAGE,"dr_node","dr"))
```

`chmod +x xxx.cfg`添加权限

#### 3.配置 CMakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)
```

```cmake
generate_dynamic_reconfigure_options(
  cfg/my_car.cfg
)
```

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo06_dynamic_reconfigure
 CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
#  DEPENDS system_lib
)
```

#### 4.编译

编译后会生成中间文件

C++ 需要调用的头文件

![](/assets/12vscode_自定义cfg的中间文件%28C++%29.PNG)

Python 需要调用的文件

![](/assets/13vscode_自定义cfg的中间文件%28Python++%29.PNG)

