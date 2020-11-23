## 6.2 URDF集成Rviz基本流程

前面介绍过，URDF 不能单独使用，需要结合 Rviz 或 Gazebo，URDF 只是一个文件，需要在 Rviz 或 Gazebo 中渲染成图形化的机器人模型，当前，首先演示URDF与Rviz的集成使用，因为URDF与Rviz的集成较之于URDF与Gazebo的集成更为简单，后期，基于Rviz的集成实现，我们再进一步介绍URDF语法。

**需求描述:**

在 Rviz 中显示一个盒状机器人

**结果演示:**![](/assets/02_URDF文件执行rviz配置02.png)

**实现流程：**

1. 准备:新建功能包，导入依赖

2. 核心:编写 urdf 文件

3. 核心:在 launch 文件集成 URDF 与 Rviz

4. 在 Rviz 中显示机器人模型

#### 1.创建功能包，导入依赖

创建一个新的功能包，名称自定义，导入依赖包:`urdf`与`xacro`

在当前功能包下，再新建几个目录:

`urdf`: 存储 urdf 文件的目录

`meshes`:机器人模型渲染文件\(暂不使用\)

`config`: 配置文件

`launch`: 存储 launch 启动文件

#### 2.编写 URDF 文件

新建一个子级文件夹:`urdf`\(可选\)，文件夹中添加一个`.urdf`文件,复制如下内容:

```xml
<robot name="mycar">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
        </visual>
    </link>
</robot>
```

#### 3.在 launch 文件中集成 URDF 与 Rviz

在`launch`目录下，新建一个 launch 文件，该 launch 文件需要启动 Rviz，并导入 urdf 文件，Rviz 启动后可以自动载入解析`urdf`文件，并显示机器人模型，核心问题:如何导入 urdf 文件? 在 ROS 中，可以将 urdf 文件的路径设置到参数服务器，使用的参数名是:`robot_description`,示例代码如下:

```xml
<launch>

    <!-- 设置参数 -->
    <param name="robot_description" textfile="$(find 包名)/urdf/urdf/urdf01_HelloWorld.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" />

</launch>
```

#### 4.在 Rviz 中显示机器人模型

rviz 启动后，会发现并没有盒装的机器人模型，这是因为默认情况下没有添加机器人显示组件，需要手动添加，添加方式如下:![](/assets/01_URDF文件执行rviz配置01.png)![](/assets/02_URDF文件执行rviz配置02.png)设置完毕后，可以正常显示了

#### 5.优化 rviz 启动

重复启动`launch`文件时，Rviz 之前的组件配置信息不会自动保存，需要重复执行步骤4的操作，为了方便使用，可以使用如下方式优化:

首先，将当前配置保存进`config`目录![](/assets/10_rviz配置保存.png)然后，`launch`文件中 Rviz 的启动配置添加参数:`args`,值设置为`-d 配置文件路径`

```xml
<launch>
    <param name="robot_description" textfile="$(find 包名)/urdf/urdf/urdf01_HelloWorld.urdf" />
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find 报名)/config/rviz/show_mycar.rviz" />
</launch>
```

再启动时，就可以包含之前的组件配置了，使用更方便快捷。

