#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from custom_msgs.msg import Point,Contour,ImagePlusTupleList
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
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Loop through each color range
    for color, (lower, upper) in color_ranges.items():
        # Create a mask for the current color range
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop through each contour
        for cnt in contours:
            # Approximate the contour to a circle
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            
            # Check if the contour is approximately circular
            if radius > 10:  # Adjust this value based on your requirements
                detected_cylinders.append((color, cnt))
    
    return detected_cylinders

class Video_Publisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(ImagePlusTupleList, '/rpi_video_feed', 10)
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
        
        col_list=[]
        msg = ImagePlusTupleList()

        for color, cnt in detected_cylinders:
            col_list.append(color)
            contour_list = [contour.tolist() for contour in cnt] #numpy array to normal
            for points_list in contour_list:
                c = Contour()
                for points in points_list:
                    p = Point()
                    p.x = points[0]
                    p.y = points[1]
                    c.points.append(p)
                msg.cnt.append(c)
                   
        
        if detected_cylinders:
            print(f"Detected {len(detected_cylinders)} cylinders")
            for color, cnt in detected_cylinders:
                print(f"Detected {color} cylinder with {len(cnt)} contour points")
        else:
            print("No cylinders detected")
   

        frame_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.image = frame_msg
        msg.col=col_list
        self.pub_.publish(msg)

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