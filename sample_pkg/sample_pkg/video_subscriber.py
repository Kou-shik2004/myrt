import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge


class Camera_sub(Node):

    def __init__(self):
        super().__init__("rpi_video_subsciber")
        self.sub_ = self.create_subscription(Image, '/rpi_video_feed', self.camCallback, 10)
        self.bridge=CvBridge()

    def camCallback(self, msg):
        frame=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('Frame',frame)
        cv2.waitKey(1)


        


def main():
    rclpy.init()

    camera_sub =Camera_sub()
    
    rclpy.spin(camera_sub)
    
    camera_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()