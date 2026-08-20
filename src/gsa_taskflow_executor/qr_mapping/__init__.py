"""二维码建图远端资源服务。

该包只管理 executor 本机 `GSA_DATA_ROOT` 下的资源索引和路径，不触碰客户端本机
文件系统。GDK 相机/控制调用仍走现有 gdk 模块和 robot_state 队列。
"""

