## 4.1 ROS元功能包

**场景:**完成一个系统性的机器人功能，工作空间下有多个功能包\(运动控制、导航、摄像头....\),为了方便管理，可以使用元功能包，

MetaPackage是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来。例如我们通过sudo apt-get install roskinetic-desktop-full 命令安装的ros就是一个虚包。我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。例如Navigation这个package就是一个MetaPackage，这个文件夹下面package.xml中的内容就是所依赖的包的名字。这样做的好处就是方便用户的安装，我们只需要这一个包就可以把其他相关的软件包组织到一起安装起来。

下面是一些常见的MetaPackage：navigation moveit! turtlebot3 ....

##### 实现流程

首先:新建一个功能包

然后,修改**package.xml**

```xml
 <exec_depend>被集成的功能包</exec_depend>
 .....
 <export>
   <metapackage />
 </export>
```

最后修改 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
catkin_metapackage()
```



