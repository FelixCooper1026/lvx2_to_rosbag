#!/usr/bin/env python3
# coding: utf-8

# 导入必要的 ROS 库
import rospy
import rosbag
import argparse
import time
import os
import struct
import numpy as np
from collections import deque
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField, PointCloud2
from math import cos, sin, radians
from tqdm import tqdm

# 导入自定义的 LVX2 解析器
from lvx2_parser import LVX2_PARSER, Frame, Package

# 将欧拉角 (度) 转换为旋转矩阵
def euler_to_rotation_matrix(roll, pitch, yaw):
    # 将角度转换为弧度
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)

    # 计算欧拉角对应的旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, cos(roll), -sin(roll)],
                    [0, sin(roll), cos(roll)]])

    R_y = np.array([[cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])

    R_z = np.array([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0, 0, 1]])

    # 合并旋转矩阵 (Z * Y * X)
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

# LVX2 到 ROS bag 转换器类
class LVX2_to_ROSBAG(object):
    def __init__(self, in_file: str, out_file: str, pc2_topic: str, pc2_frame_id: str):
        # 初始化转换器
        self._in_file = in_file                 # 输入 LVX2 文件路径
        self._out_file = out_file               # 输出 ROS bag 文件路径
        self._pc2_topic = pc2_topic             # PointCloud2 消息发布的 ROS Topic
        self._pc2_frame_id = pc2_frame_id       # PointCloud2 消息的帧 ID

        # 初始化 LVX2 解析器
        self._lvx2 = LVX2_PARSER(in_file=self._in_file)

        # 定义 PointCloud2 消息的字段结构
        # 包含 x, y, z 坐标 (float32)，intensity (float32) 和 tag (uint32)
        self._fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1), # 将 reflectivity 重命名为 intensity 以便 RViz 显示
            PointField('tag', 16, PointField.UINT32, 1),
        ]

        # 打开 ROS bag 文件进行写入
        self._bag = rosbag.Bag(self._out_file, 'w')

        # 定义点云发布的频率和缓冲参数
        self._publish_interval = 0.1  # 发布间隔，单位秒 (10 Hz)
        self._last_publish_time = 0.0 # 上次发布时间
        # 点云缓冲，用于累积多个帧的点 (每帧 50ms，10Hz 需要缓冲 2 帧，即 100ms)
        # 实际上需要缓冲来自多个设备在 100ms 内产生的所有点
        self._points_buffer = []
        self._frame_count = 0 # 用于跟踪处理的帧数

        # 处理 LVX2 文件中的帧并写入 ROS bag
        self._process_frames()

    def __del__(self):
        # 析构函数，确保关闭 ROS bag 文件
        if hasattr(self, '_bag'):
            self._bag.close()

    def _transform_points(self, points, device):
        """使用设备的外部参数对点进行变换。"""
        # 如果设备未启用外参，则直接返回原始点
        if not device.enable_extrinsic:
            return points

        # 获取旋转矩阵 (从欧拉角)
        R = euler_to_rotation_matrix(device.offset_roll,
                                   device.offset_pitch,
                                   device.offset_yaw)

        # 获取平移向量
        t = np.array([device.offset_x, device.offset_y, device.offset_z])

        # 将点数据转换为 numpy 数组以便进行向量化操作
        # points 已经是 numpy 数组 (来自 lvx2_parser)
        points_array = points # np.array(points) # 直接使用输入的 numpy 数组

        # 提取位置 (前三列) 和其他属性 (intensity, tag)
        positions = points_array[:, :3]
        intensity = points_array[:, 3]
        tag = points_array[:, 4].astype(np.uint32)  # 确保 tag 是 uint32

        # 一次性对所有位置点应用变换 (旋转和平移)
        # (R @ positions.T) 计算旋转，结果是 (3, N) 矩阵
        # .T 转置为 (N, 3)
        # + t 应用平移 (t 会自动广播)
        transformed_positions = (R @ positions.T).T + t

        # 创建包含变换后位置和原始属性的新点列表
        transformed_points = []
        for i in range(len(transformed_positions)):
            transformed_points.append([
                float(transformed_positions[i, 0]),  # x 坐标 (float)
                float(transformed_positions[i, 1]),  # y 坐标 (float)
                float(transformed_positions[i, 2]),  # z 坐标 (float)
                float(intensity[i]),                # intensity (float)
                int(tag[i])                         # tag (int)
            ])

        return transformed_points # 返回一个 Python 列表的列表

    def _process_frames(self):
        # 获取文件大小用于进度条
        file_size = os.path.getsize(self._in_file)
        start_time = time.time()

        # 创建进度条并显示转换进度、耗时和速率
        with tqdm(total=file_size, unit='B', unit_scale=True, desc="转换进度",
                 bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}, {rate_fmt}]') as pbar:

            # 遍历 LVX2 解析器流式读取的每一帧
            for frame in self._lvx2.stream_frames():
                # 更新进度条 (根据处理的帧大小)
                pbar.update(frame.header.frame_size)

                # 处理当前帧中的所有数据包
                for pkg in frame.packages:
                    # 根据数据包中的 LiDAR ID 查找对应的设备信息
                    device = next((d for d in self._lvx2._devices if d.lidar_id == pkg.header.lidar_id), None)
                    if device is None:
                        # 如果找不到对应的设备，则跳过此数据包
                        continue

                    # 获取数据包中的点数据 (已经是 numpy 数组)
                    points = pkg.points

                    # 应用外部参数变换到点数据
                    # _transform_points 返回的是一个 Python 列表的列表
                    transformed_points = self._transform_points(points, device)

                    # 将变换后的点添加到点云缓冲区
                    self._points_buffer.extend(transformed_points)

                # 增加已处理的帧计数
                self._frame_count += 1

                # 检查是否达到发布时间间隔 (每处理两帧，大约 100ms，与 10Hz 对应)
                # 并且确保缓冲区中有数据才进行发布
                # 使用帧的时间戳来决定发布时间
                current_frame_time = frame.timestamp # 帧的平均时间戳 (秒)

                # 检查是否需要发布 (当前帧时间相对于上一次发布时间)
                # 注意：这里逻辑可能需要根据实际数据包时间戳和帧组合方式微调
                # 当前简化逻辑：每处理两帧尝试发布一次缓冲区内容
                if self._frame_count % 2 == 0 and self._points_buffer:
                     # 计算当前帧的 ROS 时间戳
                     timestamp = rospy.Time.from_sec(current_frame_time)

                     # 创建 PointCloud2 消息头部
                     header = Header()
                     header.stamp = timestamp
                     header.frame_id = self._pc2_frame_id

                     # 使用缓冲区中的点创建 PointCloud2 消息
                     # self._fields 定义了点的字段结构
                     # self._points_buffer 是一个包含点数据的列表的列表
                     pc2_msg = pc2.create_cloud(header, self._fields, self._points_buffer)

                     # 将 PointCloud2 消息写入 ROS bag 文件
                     self._bag.write(self._pc2_topic, pc2_msg, timestamp)

                     # 清空缓冲区以便收集下一组帧的点
                     self._points_buffer = []


        # 文件处理完毕后，如果缓冲区中还有剩余的点，发布最后一帧
        if self._points_buffer:
             # 使用最后一帧的时间戳作为发布时间
             # 注意：这里简单使用最后一帧的时间戳，更精确可能需要计算平均值或使用结束时间
             last_frame_time = frame.timestamp if 'frame' in locals() else time.time()
             timestamp = rospy.Time.from_sec(last_frame_time)

             header = Header()
             header.stamp = timestamp
             header.frame_id = self._pc2_frame_id

             pc2_msg = pc2.create_cloud(header, self._fields, self._points_buffer)
             self._bag.write(self._pc2_topic, pc2_msg, timestamp)
             self._points_buffer = []

        # 计算并打印总转换时间
        elapsed_time = time.time() - start_time
        print(f"\n转换完成，耗时 {elapsed_time:.2f} 秒")

# 主函数
def main(args=None):
    # 创建命令行参数解析器
    _parser = argparse.ArgumentParser(description="将 lvx2 文件转换为 ROS bag 文件 (适用于 ROS1)")
    
    # 添加位置参数
    _parser.add_argument('in_file', type=str, help='输入的 lvx2 文件路径')
    _parser.add_argument('out_file', type=str, nargs='?', help='输出的 ROS bag 文件路径 (可选，默认使用输入文件名，仅将扩展名改为 .bag)')
    _parser.add_argument('pc2_topic', type=str, nargs='?', help='发布的 PointCloud2 topic 名称 (可选，默认：/livox/lidar)', default='/livox/lidar')
    _parser.add_argument('pc2_frame_id', type=str, nargs='?', help='发布的 PointCloud2 frame ID (可选，默认：livox_frame)', default='livox_frame')
    
    try:
        # 解析命令行参数
        _args = _parser.parse_args(args)

        # 检查输入文件是否存在
        if not os.path.exists(_args.in_file):
            print(f"错误: 输入文件 '{_args.in_file}' 不存在")
            return

        # 检查输入文件扩展名
        if not _args.in_file.lower().endswith('.lvx2'):
            print(f"错误: 输入文件 '{_args.in_file}' 不是有效的 LVX2 文件")
            print("提示: 输入文件必须以 .lvx2 结尾")
            return

        # 获取输入、输出文件路径和 ROS topic/frame_id
        _input_file = _args.in_file
        # 如果未指定输出文件，则根据输入文件名生成默认输出文件名
        _output_file = _args.out_file if _args.out_file else os.path.splitext(_input_file)[0] + '.bag'
        _pc2_topic = _args.pc2_topic
        _pc2_frame_id = _args.pc2_frame_id

        # 打印转换信息
        print(f"正在将 {os.path.basename(_input_file)} 转换为 ROS bag 文件: {os.path.basename(_output_file)}")
        print(f"PointCloud2 Topic: {_pc2_topic}")
        print(f"PointCloud2 Frame ID: {_pc2_frame_id}")

        # 创建并运行 LVX2 到 ROS bag 转换器
        converter = LVX2_to_ROSBAG(in_file=_input_file, out_file=_output_file, pc2_topic=_pc2_topic, pc2_frame_id=_pc2_frame_id)

    except argparse.ArgumentError as e:
        print(f"参数错误: {e}")
        _parser.print_help()
    except SystemExit as e:
        if e.code == 2:  # 参数错误
            print("\n错误: 缺少必需的输入文件参数")
            print("\n使用示例:")
            print("  基本用法:")
            print("    python3 lvx2_to_rosbag.py input.lvx2")
            print("  指定输出文件:")
            print("    python3 lvx2_to_rosbag.py input.lvx2 output.bag")
            print("  指定所有参数:")
            print("    python3 lvx2_to_rosbag.py input.lvx2 output.bag /livox/lidar livox_frame")
            print("\n使用 -h 或 --help 查看完整帮助信息")
        return
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

# 脚本的入口点
if __name__ == '__main__':
    main()
