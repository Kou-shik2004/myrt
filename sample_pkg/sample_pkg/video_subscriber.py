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

    def cam_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
        display_img = frame.copy()
        contours = msg.cnt

        # Draw a test rectangle and text
        cv2.rectangle(display_img, (50, 50), (100, 100), (0, 255, 0), 2)
        cv2.putText(display_img, "Test", (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        for contour in contours:
            cnt = np.array([(point.x, point.y) for point in contour.points], dtype=np.int32)
            cnt = cnt.reshape((-1, 1, 2))

            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            self.get_logger().info(f'Bounding box: x={x}, y={y}, w={w}, h={h}, area={area}')
            
            if area > 1000:  # Adjust this threshold as needed
                cv2.rectangle(display_img, (x,y), (x+w,y+h), (255,0,0), 2)
                cv2.putText(display_img, f'Area: {area}', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,0,0), 2)

        cv2.imshow("Detected Objects", display_img)
        cv2.waitKey(1)
    # def cam_callback(self, msg):
    #     frame = self.bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
    #     display_img = frame.copy()
    #     contours = msg.cnt
    #     self.get_logger().info(f'Number of contours received: {len(contours)}')

    #     for contour in contours:
            
    #         cnt = np.array([(point.x, point.y) for point in contour.points], dtype=np.int32)
    #         cnt = cnt.reshape((-1, 1, 2))

    #         x,y,w,h = cv2.boundingRect(cnt)
    #         self.get_logger().info(f'Bounding box: x={x}, y={y}, w={w}, h={h}, area={w*h}')
    #         if w*h > 1000:
    #             cv2.rectangle(display_img,(x,y),(x+w,y+h),(255,0,0),2)

    #     cv2.imshow("bound",display_img)

    #     cv2.waitKey(1)

def main():
    rclpy.init()
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
