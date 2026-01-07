# slamware_ros_sdk 
Currently supports both ROS1 and ROS2.

# Download the corresponding version of the ROS SDK.
Please download the relevant software packages from the official website of Slamtec.
https://www.slamtec.com/cn/support  

# Development Environment Requirements
Based on Ubuntu 20.04 / 22.04 operating system with ROS2 packages installed.
Based on Ubuntu 20.04 operating system with ROS1 packages installed.

# Hardware Requirements
To use the ROS SDK, you need an Aurora-based mapping device, powered on and configured with a suitable IP address.After slamware_ros_sdk_server_node starts, it will attempt to connect to the device.

#  install ros
1.After downloading the corresponding architecture SDK for the required version of ROS, extract it in the target environment.

# Create a workspace
1.Place the src directory containing the source code into an empty workspace directory.
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

2.Compile
    1.The compilation command for ROS 2 is
        ```bash
        cd ..
        colcon build
        ```
    2.The compilation command for ROS 1 is
        cd ..
        catkin_make


3.Configure the system environment for the workspace
    1.ros2 execute
        ```bash
        source install/setup.bash
        ```
        Alternatively, to automatically load the environment every time you start a terminal, add the following command to ~/.bashrc

        ```
        echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```

    2、ros1 execute
        ```bash
        source devel/setup.bash
        ```
        Alternatively, to automatically load the environment every time you start a terminal, add the following command to ~/.bashrc

        ```
        echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```
4.Start the node
    If the Aurora device is in AP mode, connect to the Aurora Wi-Fi and start the node
    1.ros2 
       the following commands need to be executed in sequence
    
        ```bash
        export LD_LIBRARY_PATH=~/ros_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
        
        ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1
        ```
    2.ros1 
       the following commands need to be executed
        ```bash
        
        roslaunch slamware_ros_sdk slamware_ros_sdk_server_and_view.launch ip_address:=192.168.11.1
        ```
5.For specific details regarding the aurora_ros_sdk_server_node, please refer to the relevant Wiki documentation.  
https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk/slamware_ros_sdk_server_node/

    

  










