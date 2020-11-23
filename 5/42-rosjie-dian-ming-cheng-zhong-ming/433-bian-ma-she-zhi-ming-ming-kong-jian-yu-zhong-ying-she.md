### 4.4.3 编码设置命名空间与重映射

如果自定义节点实现，那么可以更灵活的设置命名空间与重映射实现。

---

#### 1.C++ 实现:重映射

##### 1.1名称别名设置

核心代码:`ros::init(argc,argv,"zhangsan",ros::init_options::AnonymousName);`

##### 1.2执行

会在名称后面添加时间戳。

#### 2.C++ 实现:命名空间

##### 2.1命名空间设置

核心代码

```
  std::map<std::string, std::string> map;
  map["__ns"] = "xxxx";
  ros::init(map,"wangqiang");
```

##### 2.2执行

节点名称设置了命名空间。

---

#### 3.Python 实现:重映射

##### 3.1名称别名设置

核心代码:`rospy.init_node("lisi",anonymous=True)`

##### 3.2执行

会在节点名称后缀时间戳。

