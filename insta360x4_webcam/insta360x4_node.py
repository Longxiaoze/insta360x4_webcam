#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Insta360WebcamNode(Node):
    def __init__(self):
        super().__init__('insta360_webcam_node')
        # Declare parameters for device index and resolution
        self.declare_parameter('device_index', 0)  # Default camera index (0 for the first camera)
        self.declare_parameter('width', 2880)  # Default width resolution
        self.declare_parameter('height', 1440)  # Default height resolution
        
        # Retrieve parameter values
        self.device_index = self.get_parameter('device_index').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # Publisher to publish raw images to ROS topic
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        # OpenCV bridge to convert images between OpenCV and ROS formats
        self.bridge = CvBridge()

        # Open the camera and start capturing frames
        self.cap = self.open_full_pano_camera(self.device_index, self.width, self.height)

        # Timer callback to publish images every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)  

    def open_full_pano_camera(self, device_index=0, width=2880, height=1440, fourcc_str='MJPG'):
        """
        Opens the panorama camera and configures the resolution.
        
        :param device_index: Camera device index (default is 0).
        :param width: The desired resolution width (default is 2880).
        :param height: The desired resolution height (default is 1440).
        :param fourcc_str: Compression format (default is 'MJPG').
        :return: OpenCV VideoCapture object to read frames.
        """
        cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise IOError(f"Unable to open camera device {device_index}")
        
        # Set compression format (MJPG or YUYV)
        fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        
        # Set the camera resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Get the actual resolution after setting
        actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera resolution: {int(actual_w)}x{int(actual_h)}")
        
        # Check if the resolution is correctly set
        if int(actual_w) != width or int(actual_h) != height:
            self.get_logger().warn("⚠️ Resolution setting did not take effect. Please ensure camera driver and format are supported.")
        return cap

    def timer_callback(self):
        """
        Callback function that is called at regular intervals to read a frame from the camera 
        and publish it to the ROS topic.
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame, exiting.")
            return

        # Convert OpenCV image to ROS image message
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)  # Publish the image
            self.get_logger().info("Image message published.")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")


def main(args=None):
    """
    Initializes the ROS node and starts the event loop.
    """
    rclpy.init(args=args)
    node = Insta360WebcamNode()

    try:
        rclpy.spin(node)  # Keeps the node running and processing callbacks
    except KeyboardInterrupt:
        pass
    finally:
        # Release the camera and close OpenCV windows before shutting down the ROS node
        node.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
