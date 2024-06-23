#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import Point, Contour, ImagePlusTupleList
import cv2
from cv_bridge import CvBridge
import numpy as np

color_ranges = {
    "red": ([0, 100, 100], [10, 255, 255]),
    "green": ([40, 70, 50], [90, 255, 255]),
    "blue": ([90, 70, 50], [130, 255, 255]),
    "yellow": ([20, 100, 100], [30, 255, 255])
}

def detect_cylinders(frame):
    detected_cylinders = []
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    for color, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if radius > 10:
                detected_cylinders.append((color, cnt))
    return detected_cylinders

class VideoPublisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(ImagePlusTupleList, '/rpi_video_feed', 30)
        self.timer_ = self.create_timer(0.05, self.camera_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.bridge = CvBridge()

    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        detected_cylinders = detect_cylinders(frame)
        col_list = []
        msg = ImagePlusTupleList()

        for color, cnt in detected_cylinders:
            col_list.append(color)
            contour_list = [contour.tolist() for contour in cnt]
            for points_list in contour_list:
                c = Contour()
                for points in points_list:
                    p = Point()
                    p.x = points[0]
                    p.y = points[1]
                    c.points.append(p)
                msg.cnt.append(c)

        frame_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        msg.image = frame_msg
        msg.col = col_list
        self.pub_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    print("Node started")
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
