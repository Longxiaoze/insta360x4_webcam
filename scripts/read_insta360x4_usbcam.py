#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

def open_full_pano_camera(device_index=0, width=2880, height=1440, fourcc_str='MJPG'):
    """
    打开全景 USB 摄像头，并设置到指定分辨率
    :param device_index: /dev/videoX 的索引
    :param width: 期望宽度（如 2880）
    :param height: 期望高度（如 1440）
    :param fourcc_str: 摄像头支持的压缩格式（'MJPG' 或 'YUYV'）
    :return: cv2.VideoCapture 对象
    """
    # 使用 V4L2 后端打开
    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise IOError(f"无法打开摄像头设备 {device_index}")

    # 强制设置像素格式为 MJPG，以支持高分辨率
    fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # 读取回读一下，确认分辨率生效
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"摄像头实际分辨率：{int(actual_w)}x{int(actual_h)}")
    if int(actual_w) != width or int(actual_h) != height:
        print("⚠️ 注意：分辨率设置不生效，请确认摄像头驱动和格式支持。")
    return cap

def main():
    cap = open_full_pano_camera(device_index=0, width=2880, height=1440)

    cv2.namedWindow("Full Pano", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Full Pano", 1280, 640)  # 窗口显示尺寸，可以根据屏幕调整

    print("按 q 键退出")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧，退出")
            break

        # 如果是双鱼眼，可在此处直接保存／显示
        cv2.imshow("Full Pano", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
