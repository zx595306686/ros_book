### 2.4.6 rosparam

rosparam包含rosparam命令行工具，用于使用YAML编码文件在参数服务器上获取和设置ROS参数。

```
rosparam set	设置参数
rosparam get	获取参数
rosparam load	从外部文件加载参数
rosparam dump	将参数写出到外部文件
rosparam delete	删除参数
rosparam list	列出所有参数
```

* rosparam list

  列出所有参数

  ```
  rosparam list

  //默认结果
  /rosdistro
  /roslaunch/uris/host_helloros_virtual_machine__42911
  /rosversion
  /run_id
  ```

* rosparam set

  设置参数

  ```
  rosparam set name huluwa

  //再次调用 rosparam list 结果
  /name
  /rosdistro
  /roslaunch/uris/host_helloros_virtual_machine__42911
  /rosversion
  /run_id
  ```

* rosparam get

  获取参数

  ```
  rosparam get name

  //结果
  huluwa
  ```

* rosparam delete

  删除参数

  ```
  rosparam delete name

  //结果
  //去除了name
  ```

* rosparam load\(先准备 yaml 文件\)

  从外部文件加载参数

  ```
  rosparam load xxx.yaml
  ```

* rosparam dump

  将参数写出到外部文件

  ```
  rosparam dump yyy.yaml
  ```

### 



