### 8.5.2 基于rosserial\_arduino的底盘实现\_概述

在本案例中，机器人底盘\(驱动系统\)与控制系统的通信流程，前面已经多次介绍了，不再赘述，该流程程序设计上稍显复杂，Arduino端与ROS端都有相关实现，且涉及的知识点偏多，在此我们可以将流程拆分，逐步实现:

1. Arduino端先搭建框架，主要实现速度消息的订阅、时时速度的发布；
2. Arduino端再实现电机转向的控制并测试；
3. Arduino端再实现电机测速，并发布时时速度信息；
4. Arduino端将订阅的速度解析为左右电机的速度，并实现PID调速；
5. ROS端订阅速度信息，并生成里程计信息再发布。

实现流程中，大部分知识点都已经有所介绍，整个实现更偏向于知识点的集成与整合。

