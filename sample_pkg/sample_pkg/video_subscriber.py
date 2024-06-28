#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Point, Contour, ImagePlusTupleList
import numpy as np
import cv2
from cv_bridge import CvBridge

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__("rpi_video_subscriber")
        self.sub_ = self.create_subscription(ImagePlusTupleList, '/rpi_video_feed', self.cam_callback, 100)
        self.bridge = CvBridge()
        cv2.namedWindow('Cylinder Detection', cv2.WINDOW_NORMAL)

    def cam_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
        display_frame = frame.copy()
        colors = msg.col
        contours = msg.cnt

        for color, contour in zip(colors, contours):
            cnt = np.array([(point.x, point.y) for point in contour.points], dtype=np.int32)
            cnt = cnt.reshape((-1, 1, 2))
            if len(cnt) >= 4:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(display_frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                distance_text = display_frame[max(0, y-30):y, x:x+200]
                if distance_text.size > 0:
                    gray_text = cv2.cvtColor(distance_text, cv2.COLOR_BGR2GRAY)
                    _, binary_text = cv2.threshold(gray_text, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        cv2.imshow('Cylinder Detection', display_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
