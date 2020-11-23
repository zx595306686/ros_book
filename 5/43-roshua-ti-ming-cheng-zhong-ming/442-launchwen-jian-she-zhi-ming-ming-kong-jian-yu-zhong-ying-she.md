### 4.5.2 launch文件设置话题重映射

**launch 文件设置话题重映射语法:**

```
<node pkg="xxx" type="xxx" name="xxx">
    <remap from="原话题" to="新话题" />
</node>
```

实现teleop\_twist\_keyboard与乌龟显示节点通信方案由两种：

##### 1.方案1

将 teleop\_twist\_keyboard 节点的话题设置为`/turtle1/cmd_vel`

```xml
<launch>

    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key">
        <remap from="/cmd_vel" to="/turtle1/cmd_vel" />
    </node>

</launch>
```

二者可以实现正常通信

##### 2.方案2

将乌龟显示节点的话题设置为 `/cmd_vel`

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" />
    </node>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key" />

</launch>
```

二者可以实现正常通信

