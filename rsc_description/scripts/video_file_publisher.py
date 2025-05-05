#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoFilePublisher(Node):
    def __init__(self):
        super().__init__('video_file_publisher')

        # === UPDATE THIS PATH ===
        video_path = '/home/miriam/Videos/Screencasts/ANGER.mp4'
        if not os.path.exists(video_path):
            self.get_logger().error(f"Video file not found: {video_path}")
            return

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file.")
            return

        self.publisher_ = self.create_publisher(Image, '/video/image', 10)
        self.bridge = CvBridge()

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fps = fps if fps > 0 else 30.0  # fallback
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)

        self.get_logger().info("Publishing video from file to /video/image")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video ended, looping back.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoFilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
