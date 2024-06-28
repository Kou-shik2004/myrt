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

def detect_color(frame, lower_color, upper_color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return cv2.bitwise_and(frame, frame, mask=mask)

def detect_cylinders(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Detect circles (cylinder cross-sections)
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                               param1=200, param2=30, minRadius=20, maxRadius=100)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Draw the outer circle
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
            
            # Estimate distance (this is a placeholder - you'll need to calibrate this)
            distance = 1000 / i[2]  # Example formula, needs calibration
            cv2.putText(frame, f"Distance: {distance:.2f} cm", (i[0], i[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    return frame

class VideoPublisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(ImagePlusTupleList, '/rpi_video_feed', 100)
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

        detected_cylinders = detect_cylinders(frame.copy())
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
        
        if detected_cylinders:
            self.get_logger().info(f"Detected {len(detected_cylinders)} cylinders")
            for color, cnt in detected_cylinders:
                self.get_logger().info(f"Detected {color} cylinder with {len(cnt)} contour points")
        else:
            self.get_logger().info("No cylinders detected")


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
