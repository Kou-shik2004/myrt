
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class Video_Publisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(Image, '/rpi_video_feed', 10)
        timer_period = 0.5
        self.timer_ = self.create_timer(timer_period, self.cameraCallback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def cameraCallback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

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