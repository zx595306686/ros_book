### 6.6.4 Gazebo仿真环境搭建

到目前位置，我们已经可以将机器人模型显示在 Gazebo 之中了，但是当前默认情况下，在 Gazebo 中机器人模型是在 empty world 中，并没有类似于房间、家具、道路、树木... 之类的仿真物，如何在 Gazebo 中创建仿真环境呢？

Gazebo 中创建仿真实现方式有两种:

* 方式1: 直接添加内置组件创建仿真环境

* 方式2: 手动绘制仿真环境\(更为灵活\)

也还可以直接下载使用官方或第三方提高的仿真环境插件。

#### 1.添加内置组件创建仿真环境

##### 1.1启动 Gazebo 并添加组件![](/assets/19_搭建仿真环境.png)

##### 1.2保存仿真环境

添加完毕后，选择 file ---&gt; Save World as 选择保存路径\(功能包下: worlds 目录\)，文件名自定义，后缀名设置为 .world![](/assets/14_gazebo保存为world文件.png)

##### 1.3 启动

```xml
<launch>

    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find demo02_urdf_gazebo)/urdf/xacro/my_base_camera_laser.urdf.xacro" />
    <!-- 启动 gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find demo02_urdf_gazebo)/worlds/hello.world" />
    </include>

    <!-- 在 gazebo 中显示机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />
</launch>
```

核心代码: 启动 empty\_world 后，再根据`arg`加载自定义的仿真环境

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find demo02_urdf_gazebo)/worlds/hello.world" />
</include>
```

#### 2.自定义仿真环境

##### 2.1 启动 gazebo 打开构建面板，绘制仿真环境

![](/assets/12_gazebo搭建环境.png)![](/assets/13_gazebo构建环境.png)

##### 2.2 保存构建的环境

点击: 左上角 file ---&gt; Save \(保存路径功能包下的: models\)

然后 file ---&gt; Exit Building Editor

##### 2.3 保存为 world 文件

可以像方式1一样再添加一些插件，然后保存为 world 文件\(保存路径功能包下的: worlds\)

![](/assets/14_gazebo保存为world文件.png)

##### 2.4 启动

同方式1

#### 3.使用官方提供的插件

当前 Gazebo 提供的仿真道具有限，还可以下载官方支持，可以提供更为丰富的仿真实现，具体实现如下:

##### 3.1 下载官方模型库

`git clone https://github.com/osrf/gazebo_models`

之前是:`hg clone https://bitbucket.org/osrf/gazebo_models`但是已经不可用

注意: 此过程可能比较耗时

##### 3.2 将模型库复制进 gazebo

将得到的gazebo\_models文件夹内容复制到 /usr/share/gazebo-\*/models

##### 3.3 应用

重启 Gazebo，选择左侧菜单栏的 insert 可以选择并插入相关道具了

