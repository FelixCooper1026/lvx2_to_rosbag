#!/usr/bin/env python3
# coding: utf-8

# 导入必要的库
import struct
import argparse
from dataclasses import dataclass, field
import mmap
import numpy as np
from typing import Generator, Iterator, Tuple


# 定义 LVX2 文件中不同结构的数据类
@dataclass
class PublicHeader:
    # 公共头，包含文件签名、版本和魔术码
    signature  : str = '' # 文件签名，应为 'livox_tech'
    ver_a      : int = 0  # 版本号 A
    ver_b      : int = 0  # 版本号 B
    ver_c      : int = 0  # 版本号 C
    ver_d      : int = 0  # 版本号 D
    magic_code : int = 0  # 魔术码，应为 0xAC0EA767


@dataclass
class PrivateHeader:
    # 私有头，包含记录时长和设备数量
    duration     : int = 0 # 记录时长，单位 ms
    device_count : int = 0 # 设备数量


@dataclass
class DeviceInformation:
    # 设备信息，每个 LiDAR 设备对应一个
    lidar_sn         : str = '' # LiDAR 序列号
    hub_sn           : str = '' # Hub 序列号
    lidar_id         : int = 0  # LiDAR ID
    lidar_type       : int = 0  # LiDAR 类型
    device_type      : int = 0  # 设备类型
    enable_extrinsic : int = 0  # 是否启用外参
    offset_roll      : float = 0.0 # 外参：滚转角 (度)
    offset_pitch     : float = 0.0 # 外参：俯仰角 (度)
    offset_yaw       : float = 0.0 # 外参：偏航角 (度)
    offset_x         : float = 0.0 # 外参：X 轴偏移 (米)
    offset_y         : float = 0.0 # 外参：Y 轴偏移 (米)
    offset_z         : float = 0.0 # 外参：Z 轴偏移 (米)


@dataclass
class FrameHeader:
    # 帧头，描述帧在文件中的位置和大小
    current_offset : int = 0 # 当前帧在文件中的偏移量
    next_offset    : int = 0 # 下一帧在文件中的偏移量
    frame_index    : int = 0 # 帧索引
    frame_size     : int = 0 # 帧大小 (字节)


@dataclass
class PackageHeader:
    # 数据包头，描述每个数据包的信息
    version        : int   = 0 # 数据包版本
    lidar_id       : int   = 0 # LiDAR ID
    lidar_type     : int   = 0 # LiDAR 类型
    timestamp_type : int   = 0 # 时间戳类型
    timestamp      : float = 0.0 # 数据包时间戳 (ns)
    udp_count      : int   = 0 # UDP 包计数
    data_type      : int   = 0 # 点数据类型 (0x01 或 0x02)
    length         : int   = 0 # 点数据部分的长度 (字节)
    frame_count    : int   = 0 # 当前包在帧内的序号
    points_count   : int   = 0 # 点数量


@dataclass
class Point:
    # 点数据结构 (用于表示解析后的点)
    x            : float = 0.0 # X 坐标 (米)
    y            : float = 0.0 # Y 坐标 (米)
    z            : float = 0.0 # Z 坐标 (米)
    reflectivity : int   = 0 # 反射强度
    tag          : int   = 0 # 标签


@dataclass
class Package:
    # 数据包，包含包头和点数据
    header : PackageHeader = PackageHeader() # 包头
    points : list          = field(default_factory=list) # 点数据列表


@dataclass
class Frame:
    # 帧，包含帧头、时间戳和数据包
    header    : FrameHeader = FrameHeader() # 帧头
    timestamp : int         = 0 # 帧时间戳 (计算得出)
    packages  : list        = field(default_factory=list) # 数据包列表


# 从列表中弹出前 n 个元素 (FIFO)
def pop_n( size:int, data:list ):
    return ( data[:size], data[size:] )


# LVX2 文件解析器类
class LVX2_PARSER(object):
    def __init__(self, in_file: str, chunk_size: int = 1024 * 1024):
        # 初始化解析器
        self._in_file = in_file       # 输入 LVX2 文件路径
        self._chunk_size = chunk_size # 内存映射块大小
        self._pub_header = None       # 公共头
        self._prv_header = None       # 私有头
        self._devices = []            # 设备信息列表

        # 使用内存映射打开文件以提高读取效率
        self._file = open(self._in_file, 'rb')
        self._mmap = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)

        # 读取文件头部信息
        self._read_headers()

    def _read_headers(self):
        # 读取公共头
        print('==================== 公共头 ====================')
        # 尝试解析公共头，读取 24 字节
        self._pub_header = self.parse_public_header(self._mmap.read(24))
        if self._pub_header is None:
            raise ValueError("无效的公共头")

        # 读取私有头
        print('==================== 私有头 ====================')
        # 尝试解析私有头，读取 5 字节
        self._prv_header = self.parse_private_header(self._mmap.read(5))
        if self._prv_header is None:
            raise ValueError("无效的私有头")

        # 读取设备信息列表
        print('==================== 设备信息 ====================')
        # 根据私有头中的设备数量循环读取
        for i in range(self._prv_header.device_count):
            print(f'设备 #{i}')
            # 尝试解析设备信息，每个设备信息块 63 字节
            dev = self.parse_device_information(self._mmap.read(63))
            if dev is None:
                raise ValueError(f"设备 {i} 的设备信息无效")
            # 打印设备信息
            print(f'  LiDAR SN: {dev.lidar_sn}')
            # 只在 Hub SN 不为空且字符有效时打印
            if dev.hub_sn and not any(c < 32 or c > 126 for c in dev.hub_sn.encode()):
                print(f'  Hub SN: {dev.hub_sn}')
            print(f'  LiDAR ID: {dev.lidar_id}')
            print(f'  LiDAR 类型: {dev.lidar_type}')
            print(f'  设备类型: {dev.device_type}')
            print(f'  启用外参: {dev.enable_extrinsic}')
            print(f'  外参 滚转角: {dev.offset_roll}')
            print(f'  外参 俯仰角: {dev.offset_pitch}')
            print(f'  外参 偏航角: {dev.offset_yaw}')
            print(f'  外参 X 偏移: {dev.offset_x}')
            print(f'  外参 Y 偏移: {dev.offset_y}')
            print(f'  外参 Z 偏移: {dev.offset_z}')
            print('')
            # 将设备信息添加到列表中
            self._devices.append(dev)

    def stream_frames(self) -> Generator[Frame, None, None]:
        """以流式方式从文件读取帧，避免一次性加载全部数据。"""
        print('==================== 点云数据 ====================')
        # 循环读取文件直到结束
        while True:
            # 读取帧头数据 (24 字节)
            header_data = self._mmap.read(24)
            # 如果读取不到数据，表示文件已读完
            if not header_data:
                break

            # 解析帧头
            frame = Frame()
            frame.header = self.parse_frame_header(header_data)
            # 如果帧头无效则跳过当前循环
            if frame.header is None:
                continue

            # 读取帧的剩余数据 (帧大小 - 帧头大小)
            frame_data = self._mmap.read(frame.header.frame_size - 24)
            # 解析帧中的数据包
            frame.packages = self.parse_frame_packages(frame_data)

            # 计算帧的时间戳 (使用包的时间戳平均值)
            if frame.packages:
                timestamps = [pkg.header.timestamp for pkg in frame.packages]
                frame.timestamp = sum(timestamps) / len(timestamps)

            # 使用 yield 返回当前帧，实现流式处理
            yield frame

    def parse_points(self, buf: bytes, data_type: int) -> np.ndarray:
        """使用 numpy 高效解析点数据。"""
        # 根据数据类型设置点的数据长度和 numpy dtype
        if data_type == 1: # 数据类型 0x01 (14 字节/点)
            data_length = 14
            dtype = np.dtype([
                ('x', '<i4'), # X 坐标，int (4 字节)
                ('y', '<i4'), # Y 坐标，int (4 字节)
                ('z', '<i4'), # Z 坐标，int (4 字节)
                ('reflectivity', '<u1'), # 反射强度，unsigned char (1 字节)
                ('tag', '<u1') # 标签，unsigned char (1 字节)
            ])
        elif data_type == 2: # 数据类型 0x02 (8 字节/点)
            data_length = 8
            dtype = np.dtype([
                ('x', '<i2'), # X 坐标，short (2 字节)
                ('y', '<i2'), # Y 坐标，short (2 字节)
                ('z', '<i2'), # Z 坐标，short (2 字节)
                ('reflectivity', '<u1'), # 反射强度，unsigned char (1 字节)
                ('tag', '<u1') # 标签，unsigned char (1 字节)
            ])
        else:
            raise ValueError('无效的点数据类型')

        # 将字节缓冲区直接转换为结构化的 numpy 数组
        points = np.frombuffer(buf, dtype=dtype)

        # 根据数据类型确定缩放因子 (将 mm 或 cm 转换为 m)
        if data_type == 1:
            scale = 1.0e-3 # mm 转 m
        else: # data_type == 2
            scale = 1.0e-2 # cm 转 m

        # 对 x, y, z 应用缩放并转换为 float32，保留 reflectivity (float32) 和 tag (uint32)
        # 使用 np.column_stack 组合列数据
        return np.column_stack((
            points['x'].astype(np.float32) * scale,
            points['y'].astype(np.float32) * scale,
            points['z'].astype(np.float32) * scale,
            points['reflectivity'].astype(np.float32),
            points['tag'].astype(np.uint32)
        ))

    def __del__(self):
        # 析构函数，确保关闭内存映射和文件
        if hasattr(self, '_mmap'):
            self._mmap.close()
        if hasattr(self, '_file'):
            self._file.close()

    # 解析公共头
    def parse_public_header(self, buf: bytes):
        # 检查缓冲区长度是否正确
        if len(buf) != 24:
            return None

        ret = PublicHeader()
        # 解析签名 (16 字节)
        ret.signature = buf[0:16].decode(encoding='utf-8', errors='ignore')
        # 解析版本号 A, B, C, D (各 1 字节，无符号 byte) - 修正为无符号
        ret.ver_a = int(struct.unpack('<B', buf[16:17])[0])
        ret.ver_b = int(struct.unpack('<B', buf[17:18])[0])
        ret.ver_c = int(struct.unpack('<B', buf[18:19])[0])
        ret.ver_d = int(struct.unpack('<B', buf[19:20])[0])
        # 解析魔术码 (4 字节，无符号 long)
        ret.magic_code = int(struct.unpack('<L', buf[20:24])[0])

        # 签名检查
        if not ret.signature.startswith('livox_tech'):
            print(f"警告: 无效的文件签名: {ret.signature}")
            return None

        # 魔术码检查
        if ret.magic_code != 0xAC0EA767:
            print(f"警告: 无效的魔术码: {hex(ret.magic_code)}")
            return None

        return ret

    # 解析私有头
    def parse_private_header(self, buf: bytes):
        # 检查缓冲区长度是否正确
        if len(buf) != 5:
            return None

        ret = PrivateHeader()
        # 解析记录时长 (4 字节，无符号 long)，转换为秒
        ret.duration = float(struct.unpack('<L', buf[0:4])[0]) / 1.0e3  # [sec]
        # 解析设备数量 (1 字节，无符号 byte) - 修正为无符号
        ret.device_count = int(struct.unpack('<B', buf[4:5])[0])
        return ret

    # 解析设备信息
    def parse_device_information(self, buf: bytes):
        # 检查缓冲区长度是否正确
        if len(buf) != 63:
            return None

        ret = DeviceInformation()
        # 解析 LiDAR SN (16 字节)
        ret.lidar_sn = buf[0:16].decode(encoding='utf-8', errors='ignore')
        # 解析 Hub SN (16 字节)
        ret.hub_sn = buf[16:32].decode(encoding='utf-8', errors='ignore')
        # 解析 LiDAR ID (4 字节，无符号 long)
        ret.lidar_id = int(struct.unpack('<L', buf[32:36])[0])
        # 解析 LiDAR 类型 (1 字节，无符号 byte)
        ret.lidar_type = int(struct.unpack('<B', buf[36:37])[0])
        # 解析设备类型 (1 字节，无符号 byte)
        ret.device_type = int(struct.unpack('<B', buf[37:38])[0])
        # 解析是否启用外参 (1 字节，无符号 byte)
        ret.enable_extrinsic = int(struct.unpack('<B', buf[38:39])[0])
        # 解析外参：滚转角 (4 字节，float)
        ret.offset_roll = float(struct.unpack('<f', buf[39:43])[0])
        # 解析外参：俯仰角 (4 字节，float)
        ret.offset_pitch = float(struct.unpack('<f', buf[43:47])[0])
        # 解析外参：偏航角 (4 字节，float)
        ret.offset_yaw = float(struct.unpack('<f', buf[47:51])[0])
        # 解析外参：X 轴偏移 (4 字节，float)
        ret.offset_x = float(struct.unpack('<f', buf[51:55])[0])
        # 解析外参：Y 轴偏移 (4 字节，float)
        ret.offset_y = float(struct.unpack('<f', buf[55:59])[0])
        # 解析外参：Z 轴偏移 (4 字节，float)
        ret.offset_z = float(struct.unpack('<f', buf[59:63])[0])
        return ret

    # 解析帧头
    def parse_frame_header(self, buf: bytes):
        # 检查缓冲区长度是否正确
        if len(buf) != 24:
            return None

        ret = FrameHeader()
        # 解析当前帧偏移量 (8 字节，无符号 long long)
        ret.current_offset = struct.unpack('<Q', buf[0:8])[0]
        # 解析下一帧偏移量 (8 字节，无符号 long long)
        ret.next_offset = struct.unpack('<Q', buf[8:16])[0]
        # 解析帧索引 (8 字节，无符号 long long)
        ret.frame_index = struct.unpack('<Q', buf[16:24])[0]
        # 计算帧大小
        ret.frame_size = ret.next_offset - ret.current_offset
        # 检查帧大小是否有效
        if ret.frame_size <= 0:
             print(f"警告: 帧索引 {ret.frame_index} 的帧大小无效: {ret.frame_size}")
             return None
        return ret

    # 解析数据包头
    def parse_package_header(self, buf: bytes):
        # 检查缓冲区长度是否正确
        if len(buf) != 27:
            return None

        ret = PackageHeader()
        # 解析版本 (1 字节，无符号 byte)
        ret.version = int(struct.unpack('<B', buf[0:1])[0])
        # 解析 LiDAR ID (4 字节，无符号 long)
        ret.lidar_id = int(struct.unpack('<L', buf[1:5])[0])
        # 解析 LiDAR 类型 (1 字节，无符号 byte)
        ret.lidar_type = int(struct.unpack('<B', buf[5:6])[0])
        # 解析时间戳类型 (1 字节，无符号 byte)
        ret.timestamp_type = int(struct.unpack('<B', buf[6:7])[0])
        # 解析时间戳 (8 字节，无符号 long long)，转换为秒
        ret.timestamp = float(struct.unpack('<Q', buf[7:15])[0]) / 1.0e9
        # 解析 UDP Count (2 字节，无符号 short)
        ret.udp_count = int(struct.unpack('<H', buf[15:17])[0])
        # 解析数据类型 (1 字节，无符号 byte)
        ret.data_type = int(struct.unpack('<B', buf[17:18])[0])
        # 解析长度 (4 字节，无符号 long)，即点数据部分的长度
        ret.length = int(struct.unpack('<L', buf[18:22])[0])
        # 解析帧内包序号 (1 字节，无符号 byte)
        ret.frame_count = int(struct.unpack('<B', buf[22:23])[0])
        # 23-26: 保留字段 (4 字节)，忽略

        # 检查数据类型是否有效，并确定每个点的数据长度
        if ret.data_type == 1:
            data_length = 14
        elif ret.data_type == 2:
            data_length = 8
        else:
            print(f"警告: 无效的数据类型: {ret.data_type}")
            return None

        # 检查点数据长度是否与点的数据长度一致
        if ret.length % data_length != 0:
            print(f"警告: 数据包长度 {ret.length} 与点数据长度 {data_length} 不匹配")
            return None

        # 计算点数量
        ret.points_count = ret.length // data_length # 使用整数除法
        return ret

    # 解析帧中的所有数据包
    def parse_frame_packages(self, buf: bytes):
        ret = []
        # 循环处理缓冲区中的数据直到全部解析完毕
        while( len(buf) > 0 ):
            # 弹出并解析包头 (27 字节)
            (_data, buf) = pop_n(27, buf)
            pkg          = Package()
            pkg.header   = self.parse_package_header(_data)

            # 如果包头无效，则跳过当前数据包
            if pkg.header is None:
                 continue

            # 弹出并解析点数据 (长度由包头指定)
            (_data, buf) = pop_n(pkg.header.length, buf)
            # 解析点数据为 numpy 数组
            pkg.points   = self.parse_points(_data, pkg.header.data_type)

            # 将解析后的数据包添加到列表中
            ret.append(pkg)

        return ret


# 主函数
def main(args=None):
    # 创建命令行参数解析器
    _parser = argparse.ArgumentParser(description="转换 lvx2 文件到 ROS bag (适用于 ROS1) 脚本")
    # 添加输入文件参数
    _parser.add_argument('--in_file', type=str, help='输入的 lvx2 文件路径', required=True)
    # 解析命令行参数
    _args = _parser.parse_args(args)

    # 获取输入文件路径
    _input_file = _args.in_file
    # 创建 LVX2 解析器实例
    _lvx2       = LVX2_PARSER( in_file=_input_file )
    # 注意：这里只进行了文件解析，实际转换为 rosbag 的逻辑在 lvx2_to_rosbag.py 中

# 脚本的入口点
if( __name__ == '__main__' ):
    main()
