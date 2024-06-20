#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from custom_msgs.msg import Point,Contour,ImagePlusTupleList
import cv2
from cv_bridge import CvBridge


class Camera_sub(Node):

    def __init__(self):
        super().__init__("rpi_video_subsciber")
        self.sub_ = self.create_subscription(ImagePlusTupleList, '/rpi_video_feed', self.camCallback, 10)
        self.bridge=CvBridge()

    def camCallback(self, msg):
        frame=self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        color=[]
        cnt=[]
        for col in msg.col:
            color.append(col)
        cnt=msg.cnt
        print(cnt)
        for color, cnt in zip(color, cnt):

            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
       

def main():
    rclpy.init()

    camera_sub =Camera_sub()
    
    rclpy.spin(camera_sub)
    
    camera_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()