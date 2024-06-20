#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from custom_msgs.msg import Point,Contour,ImagePlusTupleList
import numpy as np
import cv2
from cv_bridge import CvBridge


class Camera_sub(Node):

    def __init__(self):
        super().__init__("rpi_video_subsciber")
        self.sub_ = self.create_subscription(ImagePlusTupleList, '/rpi_video_feed', self.camCallback, 10)
        self.bridge=CvBridge()

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

            x, y, w, h = cv2.boundingRect(cnt)
            print(f"Drawing bounding box for {color} cylinder at ({x}, {y}) with size {w}x{h}")

            if len(cnt) >= 4:  # A valid contour should have at least 4 points
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(display_frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                print(f"Invalid contour for {color} cylinder: {len(cnt)} points")

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.imshow('frame', display_frame)
        cv2.waitKey(1)
       

def main():
    rclpy.init()

    camera_sub =Camera_sub()
    
    rclpy.spin(camera_sub)
    
    camera_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()