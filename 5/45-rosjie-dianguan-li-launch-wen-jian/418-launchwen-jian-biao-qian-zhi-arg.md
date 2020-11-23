### 4.2.8 launch文件标签之arg

`<arg>`标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性

#### 1.属性

* name="参数名称"

* default="默认值" \(可选\)

* value="数值" \(可选\)

  不可以与 default 并存

* doc="描述"

  参数说明

#### 2.子级标签

* 无

#### 3.示例

* launch文件传参语法实现,hello.lcaunch

  ```xml
  <launch>
      <arg name="xxx" />
      <param name="param" value="$(arg xxx)" />
  </launch>
  ```

* 命令行调用launch传参

  ```
  roslaunch hello.launch xxx:=值
  ```



