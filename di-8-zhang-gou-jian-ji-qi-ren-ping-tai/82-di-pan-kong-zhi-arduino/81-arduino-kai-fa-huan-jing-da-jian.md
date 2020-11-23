### 8.2.1 arduino 开发环境搭建

基于Arduino的开发实现，毋庸置疑的必须先要准备Arduino电路板\(建议型号:Arduino Mega 2560\)，除了硬件之外，还需要准备软件环境，安装 Arduino IDE，在 Ubuntu 下，Arduino 开发环境的搭建步骤如下:

1. 硬件准备: Arduino电路板连接 ubuntu
2. 软件准备: 安装 Arduino IDE
3. 编写 Arduino 程序并上传至 Arduino电路板。

#### 1.Arduino 连接 Ubuntu

![](/assets/虚拟机连接arduino.png)

你需要确保你对这个接口有访问的权限。假设你的Arduino连接的是`/dev/ttyACM0`，那么就运行下面这个命令：

```
$ ls -l /dev/ttyACM0
```

然后你就可以看到类似于下面的输出结果：

> crw-rw—- 1 root dialout 166, 0 2013-02-24 08:31 /dev/ttyACM0

我们注意到在上面的结果中，只有root和”dialout”组才有读写权限。因此，你需要成为`dialout`组的一个成员。

命令如下:

```
$ sudo usermod -a -G dialout your_user_name
```

在这个命令中`your_user_name`就是你在Linux下登录的用户名。然后需要**重启**使之生效。执行完上面的操作之后，你可以运行下面的命令查看一下：

```
$ groups
```

然后如果你可以在列出的组中找到dialout，这就说明你已经加入到dialout中了。

#### 2.安装 Arduino IDE

##### 1.下载arduino ide安装包

官方下载链接：[https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software)![](/assets/arduino下载.PNG)选择对应版本即可![](/assets/arduino下载2.PNG)可以选择捐款，或者 JUST DOWNLOAD

##### 2.使用tar命令对压缩包解压

```
tar -xvf arduino-1.x.y-linux64.tar.xz
```

##### 3.将解压后的文件移动到/opt下

```
sudo mv arduino-1.x.y /opt
```

##### 4.进入安装目录,对install.sh添加可执行权限,并执行安装

```
cd /opt/arduino-1.x.y
sudo chmod +x install.sh
sudo ./install.sh
```

##### 5.启动并配置 Arduino IDE

在命令行直接输入:arduino,或者点击左下的显示应用程序搜索 arduino IDE。启动如下:![](/assets/arduino启动.PNG)Arduino IDE 配置如下:![](/assets/ArduinoIDE配置.png)

#### 3.Hello World实现

Arduino IDE 中已经内置了一些相关案例，在此，我们通过一个经典的控制 LED 等闪烁案例来演示 Arduino 的使用流程:

##### 1.案例调用![](/assets/arduino_HelloWorld.PNG)

##### 

##### 2.编译及上传

先点击左上的编译按钮，可用于语法检测，编译无异常，再点击右侧的上传按钮，上传至 Arduino 电路板![](/assets/arduino_HelloWorld2.PNG)

##### 3.运行结果

电路板上的 LED 灯闪烁

##### 4.代码解释

```cpp
// 初始化函数
void setup() {
  //将LED灯引脚(引脚值为13，被封装为了LED_BUTLIN)设置为输出模式
  pinMode(LED_BUILTIN, OUTPUT);
}

// 循环执行函数
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // 打开LED灯
  delay(1000);                       // 休眠1000毫秒
  digitalWrite(LED_BUILTIN, LOW);    // 关闭LED灯
  delay(1000);                       // 休眠1000毫秒
}
```

setup 与 loop 函数是固定格式。

