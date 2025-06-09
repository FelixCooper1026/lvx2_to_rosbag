# LVX2 to ROSBAG Converter

将 Livox LiDAR （Mid-360, HAP） 的 LVX2 点云文件转换为 ROS Bag 格式。

## 系统要求

- 操作系统：Ubuntu 20.04
- Python 版本：Python 3.7
- 其他依赖：ROS Noetic

## 安装

1. 克隆仓库：
```bash
git clone https://github.com/FelixCooper1026/lvx2_to_rosbag.git
cd lvx2_to_rosbag
```

2. 安装依赖：
```bash
pip install -r requirements.txt
```

## 使用方法

### 示例数据

1. 从以下链接下载示例数据：
   - 室内场景数据：[Indoor_sampledata.lvx2](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Indoor_sampledata.lvx2)
   - 室外场景数据：[Outdoor_sampledata.lvx2](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Outdoor_sampledata.lvx2)

2. 将示例数据放在与 Python 脚本（lvx2_to_rosbag.py）相同的文件夹中
3. 运行脚本：
```bash
python3 lvx2_to_rosbag.py --in_file ./Indoor_sampledata.lvx2 --out_file ./Indoor_sampledata.bag --pc2_topic /livox/lidar --pc2_frame_id livox_frame
```

### 参数说明

- `in_file`: 要转换的 lvx2 文件
- `out_file`: 输出的 bag 文件（可选，默认使用输入文件名，仅将扩展名改为 .bag）
- `pc2_topic`: PointCloud2 话题名称（可选，默认：/livox/lidar）
- `pc2_frame_id`: PointCloud2 数据的坐标系 ID（可选，默认：livox_frame）

## 功能特点

- 支持多设备数据解析
- 支持外参变换
- 修复点云强度（intensity）写入
- 优化点云发布频率
- 支持实时转换进度、速度、预估剩余时间显示
- 采用流式处理和内存映射，优化大文件转换性能

## 许可证

本项目采用 BSD 3-Clause 许可证。详见 [LICENSE](LICENSE) 文件。
