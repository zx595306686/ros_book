### 5.1.7 TF2与TF

#### 1.TF2与TF比较\_简介

* TF2已经替换了TF，TF2是TF的超集，建议学习 TF2 而非 TF

* TF2 功能包的增强了内聚性，TF 与 TF2 所依赖的功能包是不同的，TF 对应的是`tf`包，TF2 对应的是`tf2`和`tf2_ros`包，在 TF2 中不同类型的 API 实现做了分包处理。

* TF2 实现效率更高，比如在:TF2 的静态坐标实现、TF2 坐标变换监听器中的 Buffer 实现等

#### 2.TF2与TF比较\_静态坐标变换演示

接下来，我们通过静态坐标变换来演示TF2的实现效率。

##### 2.1启动 TF2 与 TF 两个版本的静态坐标变换

TF2 版静态坐标变换:`rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 /base_link /laser`

TF 版静态坐标变换:`rosrun tf static_transform_publisher 0 0 0 0 0 0 /base_link /laser 100`

会发现，TF 版本的启动中最后多一个参数，该参数是指定发布频率

##### 2.2运行结果比对

使用`rostopic`查看话题，包含`/tf`与`/tf_static`, 前者是 TF 发布的话题，后者是 TF2 发布的话题，分别调用命令打印二者的话题消息

`rostopic echo /tf`: 当前会循环输出坐标系信息

`rostopic echo /tf_static`: 坐标系信息只有一次

##### 2.3结论

如果是静态坐标转换，那么不同坐标系之间的相对状态是固定的，既然是固定的，那么没有必要重复发布坐标系的转换消息，很显然的，tf2 实现较之于 tf 更为高效

