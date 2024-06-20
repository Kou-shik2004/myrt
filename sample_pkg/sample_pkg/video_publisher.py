
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np

def detect_cylinders(frame):
    detected_cylinders = []
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    color_ranges = {
        "red": ((0, 100, 100), (10, 255, 255)),
        "green": ((50, 100, 100), (70, 255, 255)),
        "blue": ((100, 100, 100), (130, 255, 255)),
        # Add more color ranges as needed
    }
    # Loop through each color range
    for color, (lower, upper) in color_ranges.items():
        # Create a mask for the current color range
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop through each contour
        for cnt in contours:
            # Approximate the contour to a polygon
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            
            # Check if the polygon has the desired number of sides
            if len(approx) >= 6:  # Cylinder-like shape
                detected_cylinders.append((color, approx))
    
    return detected_cylinders

class Video_Publisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(Image, '/rpi_video_feed', 10)
        timer_period = 0.05
        self.timer_ = self.create_timer(timer_period, self.cameraCallback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.bridge = CvBridge()

    def cameraCallback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        #frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected_cylinders = detect_cylinders(frame)

        if detected_cylinders:
            print("Detected cylinders:")
        for color, approx in detected_cylinders:
            print(f"- {color} cylinder")
        else:
            print("No cylinders detected.")

        frame_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_.publish(frame_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    video_publisher = Video_Publisher()
    print("Node started")
    rclpy.spin(video_publisher)
    
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()