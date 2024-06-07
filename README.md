# Asensing_LiDAR_ROS

[English Version](README_EN.md) 

## 工程简介

本仓库是导远电子为激光雷达产品配套的 ROS SDK 驱动软件包，可运行在 Ubuntu 等 Linux 环境下。该 SDK 的核心是 `aglidar_sdk` 软件包，主要包含以下三大模块：

- 雷达驱动内核 lidar_driver
- ROS 拓展功能
- ROS2 拓展功能

如果希望基于 ROS/ROS2 进行二次开发，可以直接使用本软件包。配合 ROS/ROS2 自带的可视化工具 rviz，即可以查看点云。 

如果希望将雷达驱动集成到自己的工程，作更深一步的二次开发，请基于 lidar_driver 进行开发，它提供了最基本的功能以及一些测试工具，可运行在 Linux、Windows 等平台。

```
  +-----------------------+
  |                       |            +----------+
  |     LiDAR ROS SDK     |            |          |
  |                       |<---------->| ROS/ROS2 |
  |  +-----------------+  |            |          |
  |  |    Driver SDK   |  |            +----------+
  |  +-----------------+  |
  +-----------------------+
```

### 支持的雷达型号

Asensing LiDAR ROS SDK 目前支持两款激光雷达：

- A0（MEMS）
- A2（转镜）

### 支持的点类型

Asensing LiDAR ROS SDK 支持以下两种点类型，用户可自行扩展。

| 类型 | 字段 |
| :---: | ---- |
| XYZI | x, y, z, intensity |
| XYZIRT | x, y, z, intensity, ring, timestamp, range |


## 下载源码

### 使用 git clone 下载

使用 Git 下载 Asensing_LiDAR_ROS 项目源代码，命令如下。

```bash
$ git clone https://gitee.com/asensing/Asensing_LiDAR_ROS.git
```

注意，如果代码不全（例如缺少 lidar_driver 驱动内核），则需要执行如下命令拉取子模块。

```bash
$ cd Asensing_LiDAR_ROS
$ git submodule init
$ git submodule update
```

### 直接下载

用户可从导远电子官网，或联系技术支持，获取最新版本的 Asensing_LiDAR_ROS 源码。


## 安装依赖

### 安装 ROS 

在 ROS 环境下使用雷达驱动，需要安装 ROS 相关依赖库。

- Ubuntu 16.04 - ROS Kinetic desktop
- Ubuntu 18.04 - ROS Melodic desktop
- Ubuntu 20.04 - ROS Noetic desktop

安装方法请参考 http://wiki.ros.org。

**建议安装 ROS desktop-full 版。这个过程会自动安装一些兼容版本的依赖库，如 PCL 库等。这样可以避免花大量时间，去逐个安装和配置它们**。

### 安装 ROS2

在 ROS2 环境下使用雷达驱动，需要安装 ROS2 相关依赖库。

- Ubuntu 16.04 - 不支持
- Ubuntu 18.04 - ROS2 Eloquent desktop
- Ubuntu 20.04 - ROS2 Galactic desktop
- Ubuntu 22.04 - ROS2 Humble desktop

安装方法请参考 https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**注意：如果在一台电脑上同时安装 ROS 和 ROS2，可能会出现版本冲突问题，以及需要手工安装其他库（如 Yaml）的麻烦。**

### 安装 Yaml（必需）

版本号:  >= v0.5.2 

*若已安装 ROS desktop-full, 可跳过*

安装方法如下：

```bash
$ sudo apt-get update
$ sudo apt-get install -y libyaml-cpp-dev
```

### 安装 libpcap（必需）

版本号： >= v1.7.4

安装方法如下：

```bash
$ sudo apt-get install -y  libpcap-dev
```

## 编译、运行

可以使用三种方式编译、运行 Asensing_LiDAR_ROS 工程。

### 直接编译

(1) 打开工程内的 *CMakeLists.txt* 文件，将文件顶部的变量 **COMPILE_METHOD** 改为 **ORIGINAL**。

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD ORIGINAL)
```

(2) 在 ROS1（不适用于 ROS2）中，直接编译、运行程序。 

*(注：ROS1 不需要 aglidar_msg 文件夹，将其删掉或移动到 aglidar_sdk 文件夹内即可)*
请先启动 **roscore**，再运行 **aglidar_sdk_node**，最后运行 **rviz** 查看点云。

```bash
$ cd aglidar_sdk
$ mkdir build && cd build
$ cmake .. && make -j4
$ ./aglidar_sdk_node
```

### 基于 ROS-catkin 编译

(1) 打开工程内的 *CMakeLists.txt* 文件，将文件顶部的变量 **COMPILE_METHOD** 改为 **CATKIN**。


```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) 将 aglidar_sdk 工程目录下的 *package_ros1.xml* 文件复制到 *package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为 *src* 的文件夹, 将 aglidar_sdk 工程放入 *src* 文件夹内。(注：ROS1不需要aglidar_msg文件夹，将其删掉或移动到aglidar_sdk文件夹内即可)

(4) 返回工作空间目录，执行以下命令即可编译、运行。如果使用 .zsh，将第二行替换成 *source devel/setup.zsh*。

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch aglidar_sdk start.launch
```

### 基于 ROS2-colcon 编译

(1) 打开工程内的 *CMakeLists.txt* 文件，将文件顶部的变量 **COMPILE_METHOD** 改为 **COLCON**。

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) 将 aglidar_sdk 工程目录下的 *package_ros2.xml* 文件重命名为 *package.xml*。

(3) 返回工作空间目录，执行以下命令即可编译、运行。如果使用 .zsh，将第二行替换为 *source install/setup.zsh*。

```bash
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/setup.bash
$ ros2 launch aglidar_sdk start.py
```

(4) 若需要使用深度图，重新打开一个命令行窗口，设置环境 *source install/setup.bash*, 运行节点 *ros2 run depth_make depth_make*。

```bash
$ source install/setup.bash
$ ros2 run depth_make depth_make
```

提示：若 colcon build 时提示 CMake Error at CMakeLists.txt:45(if)，执行 `source /opt/ros/humble/setup.bash` 即可。

（6）若需要将 rosbag 

另外，不同 ROS2 版本 start.py 的格式可能不同，请使用对应版本的 start.py。如 ROS2 Elequent，请使用 elequent_start.py。

安装 ROS2 可参考 [如何安装 ROS2](doc/howto/04_install_ROS2_CN.md)。


## 参数介绍

LiDAR ROS SDK 通过读取配置文件 config.yaml，得到所有的参数。config.yaml 在 aglidar_sdk/config 文件夹中。

> 提示：config.yaml 遵循 YAML 格式。该格式对缩进有严格要求。修改 config.yaml 之后，请确保每行开头的缩进仍保持一致！

config.yaml 包括两部分：common 部分和 lidar 部分。

### common 参数

common 部分设置雷达消息的源（Packet 或点云从哪来）和目标（Packet 或点云发布到哪去）。

```yaml
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: false
```

- **msg_source**
  - 1 – 连接在线雷达。
  - 2 – 离线解析 ROS/ROS2 的 Packet 包。
  - 3 – 离线解析 PCAP 包。
- **send_packet_ros**
  - true – 雷达 Packet 消息将通过 ROS/ROS2 发出。*雷达 ROS packet 消息为自定义 ROS 消息，用户使用 ROS/ROS2 echo 命令不能查看消息的具体内容。这个功能用于录制 ROS/ROS2 的 Packet 包，更多使用细节，请参考 msg_source=2 的情况。*
- **send_point_cloud_ros**
  - true – 雷达点云消息将通过 ROS/ROS2 发出。*点云消息的类型为 ROS 官方定义的点云类型 sensor_msgs/PointCloud2, 用户可以使用 Rviz 直接查看点云。用户可以录制 ROS/ROS2 的点云包，但点云包的体积非常大，所以不建议这么做。更好的方式是录制 Packet 包，请参考 send_packet_ros=true 的情况。*

### lidar 参数

lidar 部分根据每个雷达的实际情况进行设置。

```yaml
lidar:
  - driver:
      lidar_type: A0
      msop_port: 51180
      difop_port: 7788
      min_distance: 0.2
      max_distance: 200
      use_lidar_clock: false
      write_pkt_ts: false
      dense_points: false
      pcap_path: /home/asensing/lidar.pcap
    ros:
      ros_send_by_rows: false
      ros_frame_id: aglidar
      ros_recv_packet_topic: /aglidar_packets
      ros_send_packet_topic: /aglidar_packets
      ros_send_point_cloud_topic: /aglidar_points
```

- **lidar_type**
  支持的雷达型号在 aglidar_sdk 的 README 文件中列出。
- **msop_port**, **difop_port**
  接收 MSOP/DIFOP Packet 的 msop 端口号和 difop 端口号。若收不到消息，请优先确认这两个参数是否配置正确。
- **min_distance**, **max_distance**
  点云的最小距离和最大距离。这个设置是软件屏蔽，会将区域外的点设置为 NAN 点，不会减小每帧点云的体积。
- **use_lidar_clock**
  - true – 使用雷达时间作为消息时间戳。
  - false – 使用电脑主机时间作为消息时间戳。
- **write_pkt_ts**
  - true – 将电脑主机时间写入 pakcets（前提：use_lidar_clock 为 false）。
  - false – 不将电脑主机时间写入 pakcets。
- **dense_points** 输出的点云中是否剔除 NAN points。默认值为 false。
  - true – 为剔除，
  - false – 为不剔除。
- **pcap_path**
  pcap 包的路径。当 msg_source=3 时有效。
- **ros_send_by_rows** 只有当 dense_points = false 时才有效。
  - true – 发送点云时，按照一行一行的顺序排列点。
  - false – 发送点云时，按照一列一列的顺序排列点。


## rosbag 的录制和回放

### 录制 rosbag

这里假设已经连接在线雷达，并能发送点云到 ROS。

```yaml
common:
  msg_source: 1
  send_packet_ros: true
  send_point_cloud_ros: true
  send_packet_proto: false
  send_point_cloud_proto: false
```

要录制 Packet，首先需要设置 send_packet_ros = true。

修改 ros_send_packet_topic，来改变发送的话题。这个话题包括 MSOP Packet 和 DIFOP Packet。

```yaml
ros:
  ros_frame_id: aglidar
  ros_recv_packet_topic: /aglidar_packets
  ros_send_packet_topic: /aglidar_packets
  ros_send_point_cloud_topic: /aglidar_points
```

ROS 录制 rosbag 的指令如下：

```bash
rosbag record <topic> -O bag
```

ROS2 录制 rosbag 的指令如下：

```bash
ros2 bag record <topic>
```

### 回放 rosbag

首先需要配置 Packet 源，假设录制了一个 rosbag，其中包含话题为 /aglidar_packets 的点云 Packet。

配置 config.yaml 的 common 部分。

```yaml
common:
  msg_source: 2
  send_packet_ros: false
  send_point_cloud_ros: true
  send_packet_proto: false
  send_point_cloud_proto: false
```

点云 Packet 来自 ROS rosbag，因此设置 msg_source = 2。
将点云发送到 ROS，因此设置 send_point_cloud_ros = true。
然后设置 LiDAR 类型，配置 config.yaml 的 lidar-driver 部分，将 lidar_type 设置为 LiDAR 类型。

```yaml
lidar:
  - driver:
      lidar_type: A0
      msop_port: 51180
      difop_port: 7788
      min_distance: 0.2
      max_distance: 200
      use_lidar_clock: false
      write_pkt_ts: false
```

设置接收 Packet 的主题，设置 config.yaml 的 lidar-ros 部分。

```yaml
ros:
  ros_frame_id: aglidar
  ros_recv_packet_topic: /aglidar_packets
  ros_send_packet_topic: /aglidar_packets
  ros_send_point_cloud_topic: /aglidar_points
```

将 ros_recv_packet_topic 设置为 rosbag 中点云 Packet 的话题。

运行程序，回放 rosbag。（注：需要先运行程序）

```yaml
# ROS1
rosbag play <bag_file_name>

# ROS2
ros2 bag play <bag_file_name>
```


## FAQs

### 启动程序后看不到 Rviz 界面？

检查是否正确安装对应 ROS 版本的 Rviz 软件包，可通过以下命令安装或者重新安装。

```bash
sudo apt install ros-${ROS_DISTRO}-rviz
```

注意：上述命令需要先启动 ROS 的环境变量才能看到 ROS_DISTRO 环境变量，你也可以直接指定 ROS 的版本，例如 ros-noetic-rviz。

### 找不到 rostopic 命令？

rostopic 是 ROS1 特有的命令，对应 ROS2 的 ros2 topic 命令。因此，首先需要确定你是否在使用 ROS1 环境，如果是，可通过以下命令安装 rostopic 命令工具。

```bash
$ sudo apt install python3-rostopic
```

### 找不到 colcon 命令？

Colcon 编译系统需要额外安装，安装命令如下：

```bash
sudo apt install python3-colcon-common-extension
```

### 如何将 rosbag 转换成 pcd 格式

SDK 中包含了一个 pcl_ros 工具包，支持将 rosbag 数据转成 pcd 格式。具体操作方法是先启动 pcl_ros 包中的 bag_to_pcd 节点。

```bash
ros2 run pcl_ros bag_to_pcd
```

此时，bag_to_pcd 节点会订阅 /aglidar_points 主题，等待点云数据。因此，当你回放 rosbag 时，就会对每一帧数据进行格式转换，并保存到当前目录中。

实际上，你也可以使用实时雷达，或者 pcap 数据作为数据源，只要发布 /aglidar_points 主题点云数据，即可完成 PCD 格式转换。
