# LVX2 到 ROS Bag 转换工具

## 项目简介

本项目是一个用于将 Livox LiDAR 的 LVX2 格式点云文件转换为 ROS Bag 文件的 Python 工具。它能够解析 LVX2 文件中的点云数据、设备信息和外部参数，并将点云数据以及应用了外部参数变换后的点云数据发布到 ROS Bag 中，方便在 ROS 环境中进行后续处理和可视化（如使用 RViz）。

本项目在原有转换代码的基础上进行了性能优化（使用内存映射和 NumPy 向量化操作）和功能增强（支持多设备外参变换、添加转换进度条），旨在提供一个高效、实用的 LVX2 到 ROS Bag 转换解决方案。

## 主要特性

*   支持解析 LVX2 文件的公共头、私有头、设备信息、帧头和数据包。
*   支持解析 LVX2 文件中的点云数据（数据类型 0x01 和 0x02）。
*   应用 LiDAR 设备的外部参数（滚转角、俯仰角、偏航角及 XYZ 偏移）对点云数据进行坐标变换。
*   将变换后的点云数据打包成 ROS `sensor_msgs/PointCloud2` 消息。
*   将 PointCloud2 消息写入指定的 ROS Bag 文件。
*   采用流式处理和内存映射，优化大文件转换的性能。
*   使用 `tqdm` 库显示转换进度、速率和预估剩余时间。
*   支持通过命令行参数指定输入文件、输出文件、PointCloud2 的 Topic 名称和 Frame ID。
*   将点云发布频率固定为 10Hz，每个发布的点云包含约 100ms 内的数据。

## 系统要求

*   Ubuntu 操作系统 (推荐，ROS 主要运行环境)
*   Python 3
*   已安装 ROS (Melodic, Noetic 或其他兼容版本)
*   依赖库：`rospy`, `rosbag`, `numpy`, `tqdm`

## 安装步骤

1.  确保您的系统中已正确安装 ROS 和 Python 3。
2.  将本项目代码克隆或下载到您的 ROS 工作空间 (catkin workspace) 的 `src` 目录下。
    ```bash
    cd your_ros_ws/src
    git clone <本项目仓库地址> # 如果您使用 Git
    # 或者手动下载代码到此处
    ```
3.  安装 Python 依赖库。如果您使用 pip：
    ```bash
    pip install numpy tqdm
    ```
    如果您在 ROS 环境中使用系统自带的 Python，可能需要使用 `pip3` 或通过 apt 安装：
    ```bash
    # 示例 (根据您的 ROS 版本和 Python 版本可能有所不同)
    sudo apt-get update
    sudo apt-get install python3-numpy python3-tqdm
    ```
4.  回到工作空间根目录并编译 (catkin_make)：
    ```bash
    cd your_ros_ws
    catkin_make
    # 或 catkin build (如果使用 catkin tools)
    ```
5.  激活工作空间的环境变量：
    ```bash
    source devel/setup.bash
    # 如果您使用 catkin tools:
    # source install/setup.bash
    ```

## 使用方法

确保您的 ROS 环境已激活，然后在终端中运行脚本：

```bash
rosrun your_package_name lvx2_to_rosbag.py --in_file <输入的LVX2文件路径> [可选参数]
```

**命令行参数：**

*   `--in_file <路径>`：**必需**。输入的 LVX2 文件路径。
*   `--out_file <路径>`：可选。输出的 ROS Bag 文件路径。如果未指定，默认为输入文件同名但扩展名为 `.bag`。
*   `--pc2_topic <话题名>`：可选。发布的 PointCloud2 Topic 名称。默认为 `/livox/lidar`。
*   `--pc2_frame_id <帧ID>`：可选。发布的 PointCloud2 消息的 Frame ID。默认为 `livox_frame`。

**示例：**

将名为 `input.lvx2` 的文件转换为 `output.bag`，并发布到 `/livox/pointcloud` Topic，使用 `base_link` 作为 Frame ID：

```bash
rosrun your_package_name lvx2_to_rosbag.py --in_file /path/to/input.lvx2 --out_file /path/to/output.bag --pc2_topic /livox/pointcloud --pc2_frame_id base_link
```

将名为 `data.lvx2` 的文件转换为默认名称 (`data.bag`)，使用默认 Topic 和 Frame ID：

```bash
rosrun your_package_name lvx2_to_rosbag.py --in_file /path/to/data.lvx2
```

转换过程中将显示进度条。

## 代码结构

*   `lvx2_parser.py`: 负责解析 LVX2 文件的低级别结构和数据。
*   `lvx2_to_rosbag.py`: 负责读取解析后的数据，应用外参变换，缓冲点云并将其打包为 PointCloud2 消息写入 ROS Bag 文件。

## 许可证

[BSD 3-Clause License](LICENSE)
