### 6.3.5 URDF工具

在 ROS 中，提供了一些工具来方便 URDF 文件的编写，比如:

* `check_urdf`命令可以检查复杂的 urdf 文件是否存在语法问题

* `urdf_to_graphiz`命令可以查看 urdf 模型结构，显示不同 link 的层级关系

当然，要使用工具之前，首先需要安装，安装命令:`sudo apt install liburdfdom-tools`

#### 1.check\_urdf 语法检查

进入urdf文件所属目录，调用:`check_urdf urdf文件`，如果不抛出异常，说明文件合法,否则非法

![](/assets/03_URDF文件检查_正常.png)

![](/assets/04_URDF文件检查_异常.png)

#### 2.urdf\_to\_graphiz 结构查看

进入urdf文件所属目录，调用:`urdf_to_graphiz urdf文件`，当前目录下会生成 pdf 文件

![](/assets/05_查看URDF文件模型结构.png)

