### 4.6.2 launch文件设置参数

通过 launch 文件设置参数的方式前面已经介绍过了，可以在 node 标签外，或 node 标签中通过 param 或 rosparam 来设置参数。在 node 标签外设置的参数是全局性质的，参考的是 / ，在 node 标签中设置的参数是私有性质的，参考的是 /命名空间/节点名称。

#### 1.设置参数

以 param 标签为例，设置参数

```xml
<launch>

    <param name="p1" value="100" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <param name="p2" value="100" />
    </node>

</launch>
```

#### 2.运行

`rosparam list`查看节点信息,显示结果:

```
/p1
/t1/p1
```

运行结果与预期一致。

