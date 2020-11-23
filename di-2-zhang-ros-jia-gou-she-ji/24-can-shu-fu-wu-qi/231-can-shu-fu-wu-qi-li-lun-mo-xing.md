### 2.3.1 参数服务器理论模型

参数服务器实现是最为简单的，该模型如下图所示,该模型中涉及到三个角色:

* ROS Master \(管理者\)
* Talker \(参数设置者\)
* Listener \(参数调用者\)

ROS Master 作为一个公共容器保存参数，Talker 可以向容器中设置参数，Listener 可以获取参数。

![](/assets/03ROS通信机制03_参数服务器.jpg)

整个流程由以下步骤实现:

#### 1.Talker 设置参数

Talker 通过 RPC 向参数服务器发送参数\(包括参数名与参数值\)，ROS Master 将参数保存到参数列表中。

#### 2.Listener 获取参数

Listener 通过 RPC 向参数服务器发送参数查找请求，请求中包含要查找的参数名。

#### 3.ROS Master 向 Listener 发送参数值

ROS Master 根据步骤2请求提供的参数名查找参数值，并将查询结果通过 RPC 发送给 Listener。

参数可使用数据类型:

* 32-bit integers

* booleans

* strings

* doubles

* iso8601 dates

* lists

* base64-encoded binary data

* 字典

> 注意:参数服务器不是为高性能而设计的，因此最好用于存储静态的而二进制的简单数据



