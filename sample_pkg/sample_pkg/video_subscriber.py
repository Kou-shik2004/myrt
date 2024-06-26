#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Point, Contour, ImagePlusTupleList
import numpy as np
import cv2
from cv_bridge import CvBridge
color_dict = {
        "red": (0, 0, 255),
        "green": (0, 255, 0),
        "blue": (255, 0, 0),
        "yellow": (0, 255, 255)
    }
class CameraSubscriber(Node):

    def __init__(self):
        super().__init__("rpi_video_subscriber")
        self.sub_ = self.create_subscription(ImagePlusTupleList, '/rpi_video_feed', self.cam_callback, 1000)
        self.bridge = CvBridge()


    def cam_callback(self, msg):

        frame = self.bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
        
        display_img = frame.copy()
        
        contours = msg.cnt
        self.get_logger().info(f'Number of contours received: {len(contours)}')

        for contour in contours:
            
            cnt = np.array([(point.x, point.y) for point in contour.points], dtype=np.int32)
            cnt = cnt.reshape((-1, 1, 2))

            x,y,w,h = cv2.boundingRect(cnt)
            area = w * h
            self.get_logger().info(f'{contour.color.capitalize()} bounding box: x={x}, y={y}, w={w}, h={h}, area={area}')
            
            color = color_dict.get(contour.color, (255, 255, 255))  # Default to white if color not found
            cv2.rectangle(display_img, (x,y), (x+w,y+h), color, 2)
            cv2.putText(display_img, f'{contour.color}: {area}', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

           
        
        cv2.imshow("bound",display_img)

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
