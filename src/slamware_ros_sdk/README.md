# slamware_ros_sdk 
目前支持ros1和ros2

# 下载相关版本的ros sdk
请在思岚科技官方网站上进行相关的软件包下载  https://www.slamtec.com/cn/support  

# 开发环境需求
1、基于Ubuntu 20.04 / 22.04操作系统，并装有ROS2软件包。
2、基于Ubuntu 20.04 操作系统，并装有ROS1软件包

# 硬件需求
为使用ROS SDK，您需要一台基于Auraro空间建图设备，开启并配置合适的IP地址。slamware_ros_sdk_server_node节点启动后将尝试连接该设备。



# 安装ros
1、根据需要下载好对应版本ros的对应架构的sdk后，在使用环境内先进行解压缩

# 创建工作空间
1.将存放源码的src放入一个空的工作目录

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

2、编译
    1、ros2 的编译命令
        ```bash
        cd ..
        colcon build
        ```
    2、ros1 的编译命令
        cd ..
        catkin_make


3、配置工作空间系统环境
    1、ros2 执行
        ```bash
        source install/setup.bash
        ```
        或者为了每次启动终端时自动加载环境： 将以下命令添加到 `~/.bashrc` 中

        ```
        echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```

    2、ros1 执行
        ```bash
        source devel/setup.bash
        ```
        或者为了每次启动终端时自动加载环境: 将以下命令添加到 `~/.bashrc` 中

        ```
        echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```
4、 启动节点
    若Aurora设备处于AP模式，连接Aurora WIFI，启动节点
    1、ros2 需要先后执行如下命令行
    
        ```bash
        export LD_LIBRARY_PATH=~/ros_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
        
        ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
        ```
    2、ros1 需要执行如下命令行
        ```bash
        
        roslaunch slamware_ros_sdk slamware_ros_sdk_server_and_view.launch ip_address:=192.168.11.1
        ```
5、具体相关的aurora_ros_sdk_server_node节点可以查看相关的Wiki说明 
https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/

    

  










