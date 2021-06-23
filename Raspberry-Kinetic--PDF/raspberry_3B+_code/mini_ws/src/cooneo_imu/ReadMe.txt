cooneo_imu的使用方法：
    1、首先需要使用一个 USB-TTL模块，将传感器连接到电脑上（注意:rx<-->tx、tx<-->rx），然后在终端对电脑给其分配的端口号赋权限：
        sudo chmod 0666 /dev/ttyUSB*;
    2、安装imu_tools工具，对解算的imu数据进行滤波，忽略掉数据毛刺。可安装源码，或者是二进制文件：
	源码链接：https://github.com/ccny-ros-pkg/imu_tools.git 
		在根目录下安装相关的依赖包：rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y  
    		然后将该功能包移动至工作空间，编译并source环境。
	二进制安装：sudo apt-get install ros-kinetic-imu-tools
    3、在终端输入：roslaunch cooneo_imu cooneo_imu.launch即可在rviz中查看该imu模块的可视化信息。

注：1、如果需要使用imu——tools功能包，那么就需要去掉 cooneo_imu.launch文件中的相关 node 注释。 
    2、因为每启动一次程序就会先校准，消除零偏差。在测试使用该imu模块的时候，应该将其平放，固定。
