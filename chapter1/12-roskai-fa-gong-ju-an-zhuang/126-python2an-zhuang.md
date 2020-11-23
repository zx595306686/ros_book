### 1.2.6 Python2安装

在ROS当前版本:Noetic 中已经可以支持 python3 了，但是后续使用的功能包具有滞后性，可能需要python2的支持，因此需要安装python2

启用 universe 源仓库：

```
sudo add-apt-repository universe
```

更新软件包索引，并且安装 Python 2：

```
sudo apt update 
sudo apt install python2
```

使用`curl`命令来下载`get-pip.py`脚本：

```
curl https://bootstrap.pypa.io/get-pip.py --output get-pip.py
```

一旦源仓库被启用，以 sudo 用户身份使用`python2`运行脚本来为 Python 2 安装 pip：

```
sudo python2 get-pip.py
```

测试:

```
pip2 --version
```



