### 4.2.3 launch文件标签之include

`include`标签用于将另一个 xml 格式的 launch 文件导入到当前文件

#### 1.属性

* file="$\(find 包名\)/xxx/xxx.launch"

  要包含的文件路径

* ns="xxx" \(可选\)

  在指定命名空间导入文件

#### 2.子级标签

* env 环境变量设置

* arg 将参数传递给被包含的文件



