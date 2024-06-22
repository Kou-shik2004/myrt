#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from custom_msgs.msg import Point, Contour, ImagePlusTupleList
import numpy as np
import cv2
from cv_bridge import CvBridge

class Camera_sub(Node):

    def __init__(self):
        super().__init__("rpi_video_subscriber")
        self.sub_ = self.create_subscription(ImagePlusTupleList, '/rpi_video_feed', self.camCallback, 10)
        self.bridge = CvBridge()

    def camCallback(self, msg):
        print(f"Received message with {len(msg.col)} colors and {len(msg.cnt)} contours")
        frame = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        display_frame = frame.copy()
        colors = msg.col
        contours = msg.cnt

        for color, contour in zip(colors, contours):
            # Convert the points to the format expected by cv2.boundingRect
            cnt = np.array([(point.x, point.y) for point in contour.points], dtype=np.int32)
            
            # Reshape the array to the format expected by cv2.boundingRect
            cnt = cnt.reshape((-1, 1, 2))

            if len(cnt) >= 4:  # A valid contour should have at least 4 points
                x, y, w, h = cv2.boundingRect(cnt)
                print(f"Drawing bounding box for {color} cylinder at ({x}, {y}) with size {w}x{h}")
                
                # The bounding box is already drawn by the publisher, so we don't need to draw it again
                # We'll just add the color label
                cv2.putText(display_frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Extract the distance information from the image
                distance_text = display_frame[y-30:y, x:x+200]  # Adjust these values if needed
                gray_text = cv2.cvtColor(distance_text, cv2.COLOR_BGR2GRAY)
                _, binary_text = cv2.threshold(gray_text, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                
                # Use OCR to extract the distance (you might need to install pytesseract)
                # import pytesseract
                # distance = pytesseract.image_to_string(binary_text)
                # print(f"Detected distance for {color} cylinder: {distance}")
                
                # For now, we'll just print that we detected the distance text
                print(f"Detected distance text for {color} cylinder")
            else:
                print(f"Invalid contour for {color} cylinder: {len(cnt)} points")

        cv2.namedWindow('Cylinder Detection', cv2.WINDOW_NORMAL)
        cv2.imshow('Cylinder Detection', display_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()

    camera_sub = Camera_sub()
    
    rclpy.spin(camera_sub)
    
    camera_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()