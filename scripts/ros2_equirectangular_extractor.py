#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_image_to_mp4.py

从 ROS 2 Humble 的 rosbag2 文件中提取指定 Image 话题的所有帧，
并将其保存为一个 MP4 视频文件。

用法示例：
    python3 ros2_image_to_mp4.py \
        --bag ./my_bag_folder \
        --topic /camera/color/image_raw \
        --output ./output_video.mp4 \
        --fps 25
"""

import os
import argparse

import cv2
import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    parser = argparse.ArgumentParser(
        description="从 ROS2 Humble rosbag2 中提取 Image 话题并保存为 MP4 视频"
    )
    parser.add_argument(
        "--bag", "-b", required=True,
        help="rosbag2 存放目录或文件路径（SQLite3 格式）"
    )
    parser.add_argument(
        "--topic", "-t", required=True,
        help="要提取的 Image 话题名称，例如 /camera/color/image_raw"
    )
    parser.add_argument(
        "--output", "-o", required=True,
        help="输出 MP4 视频文件路径，例如 ./video.mp4"
    )
    parser.add_argument(
        "--fps", "-f", type=float, default=30.0,
        help="生成视频的帧率，默认为 30.0"
    )
    args = parser.parse_args()

    # 初始化 ROS2 客户端（虽然不启动节点，但必须调用以加载消息类型）
    rclpy.init(args=None)

    # 设置 rosbag2_reader
    storage_options = StorageOptions(uri=args.bag, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 查找所有可用话题的元信息
    topic_types = reader.get_all_topics_and_types()
    # 确保用户要的 topic 存在且类型为 sensor_msgs/msg/Image
    if not any(t.name == args.topic and t.type == 'sensor_msgs/msg/Image' for t in topic_types):
        raise RuntimeError(f"在 rosbag 中未找到 Image 话题: {args.topic}")

    bridge = CvBridge()
    video_writer = None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    print(f"[+] 开始从 rosbag 提取话题 {args.topic}，输出到 {args.output} ...")

    # 循环读取
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic != args.topic:
            continue

        # 反序列化成 ROS Image 消息
        img_msg = deserialize_message(data, Image)
        # 转为 OpenCV 图像
        cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # 第一次初始化 VideoWriter（需要图像尺寸信息）
        if video_writer is None:
            height, width = cv_img.shape[:2]
            video_writer = cv2.VideoWriter(args.output, fourcc,
                                           args.fps, (width, height))
            if not video_writer.isOpened():
                raise RuntimeError("无法打开视频写入器，请检查输出路径和格式。")

        # 写入一帧
        video_writer.write(cv_img)

    # 结束并释放资源
    if video_writer:
        video_writer.release()
    rclpy.shutdown()

    print("[+] 完成：视频已保存。")

if __name__ == '__main__':
    main()
