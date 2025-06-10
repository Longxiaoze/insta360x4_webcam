#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_dualfisheye_extractor_with_video.py

从 ROS 2 Humble 的 rosbag2 文件中提取 dual-fisheye 图像，
同时保存原图、左右两侧裁剪图，并将三路视频写入磁盘。

用法示例：
    python3 ros2_dualfisheye_extractor_with_video.py \
        --bag ./my_bag_folder \
        --topic /camera/dual_fisheye/image_raw \
        --output_dir ./output_images \
        --fps 30
"""

import os
import argparse

import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    parser = argparse.ArgumentParser(
        description="从 ROS2 Humble rosbag 中提取 dual-fisheye 图像并保存视频"
    )
    parser.add_argument(
        "--bag", "-b", required=True,
        help="rosbag2 存放目录或文件路径（SQLite3）"
    )
    parser.add_argument(
        "--topic", "-t", default="/camera/dual_fisheye/image_raw",
        help="dual-fisheye 图像话题名称"
    )
    parser.add_argument(
        "--output_dir", "-o", default="output_images",
        help="输出根目录，脚本会在此下创建子目录"
    )
    parser.add_argument(
        "--fps", type=float, default=30.0,
        help="输出视频的帧率 (default: 30)"
    )
    args = parser.parse_args()

    # 创建输出子目录
    full_dir  = os.path.join(args.output_dir, "full_2880x1440")
    left_dir  = os.path.join(args.output_dir, "left_1440x1440")
    right_dir = os.path.join(args.output_dir, "right_1440x1440")
    video_dir = os.path.join(args.output_dir, "save_videos")
    for d in (full_dir, left_dir, right_dir):
        os.makedirs(d, exist_ok=True)

    # 初始化 VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    full_video_path  = os.path.join(video_dir,  'insta360x4-calib3-dual-fisheye_full.mp4')
    left_video_path  = os.path.join(video_dir,  'insta360x4-calib3-dual-fisheye_front.mp4')
    right_video_path = os.path.join(video_dir, 'insta360x4-calib3-dual-fisheye_back.mp4')
    full_writer  = cv2.VideoWriter(full_video_path,  fourcc, args.fps, (2880, 1440))
    left_writer  = cv2.VideoWriter(left_video_path,  fourcc, args.fps, (1440, 1440))
    right_writer = cv2.VideoWriter(right_video_path, fourcc, args.fps, (1440, 1440))

    # 打开 rosbag
    reader = SequentialReader()
    storage_opts   = StorageOptions(uri=args.bag, storage_id='sqlite3')
    conv_opts      = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_opts, conv_opts)

    bridge = CvBridge()

    print(f"开始读取 {args.bag} 上的 `{args.topic}` 话题，保存到 {args.output_dir} ...")
    count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue

        # 反序列化为 Image 消息
        img_msg = deserialize_message(data, Image)
        # 转为 OpenCV 格式
        cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        h, w = cv_img.shape[:2]
        if (w, h) != (2880, 1440):
            # 分辨率不符则跳过
            continue

        # 用 header.stamp 构造文件名
        ts = f"{img_msg.header.stamp.sec:010d}_{img_msg.header.stamp.nanosec:09d}"
        fname = f"{ts}.png"

        # 保存整图和裁剪图像
        cv2.imwrite(os.path.join(full_dir,  fname), cv_img)
        left  = cv_img[:, :1440]
        right = cv_img[:, 1440:]
        cv2.imwrite(os.path.join(left_dir,  fname), left)
        cv2.imwrite(os.path.join(right_dir, fname), right)

        # 写入视频
        full_writer.write(cv_img)
        left_writer.write(left)
        right_writer.write(right)

        count += 1

    # 释放 VideoWriter
    full_writer.release()
    left_writer.release()
    right_writer.release()

    print(f"共保存 {count} 帧图像和视频到 `{args.output_dir}`")

if __name__ == "__main__":
    main()
