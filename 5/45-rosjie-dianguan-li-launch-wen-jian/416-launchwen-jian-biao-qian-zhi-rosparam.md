### 4.2.6 launch文件标签之rosparam

`<rosparam>`标签可以从 YAML 文件导入参数，或将参数到处到 YAML 文件，也可以用来删除参数，`<rosparam>`标签在`<node>`标签中时被视为私有

#### 1.属性

* command="load \| dump \| delete" \(可选，默认 load\)

  加载、导出或删除参数

* file="$\(find xxxxx\)/xxx/yyy...."

  加载或导出到的 yaml 文件

* param="参数名称"

* ns="命名空间" \(可选\)

#### 2.子级标签

* 无



