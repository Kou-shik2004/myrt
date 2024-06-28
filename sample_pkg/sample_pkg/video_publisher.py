#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Point, Contour, ImagePlusTupleList
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

# Loading caliberation data

calibdata_path = '/home/pi/myrt_ws/src/sample_pkg/sample_pkg/calibdata.npz'
print(f"Loading calibration data from: {os.path.abspath(calibdata_path)}")
calibdata = np.load(calibdata_path)
print(calibdata.files)  # Print the keys to verify

mtx = calibdata['mtx']
dist = calibdata['dist']
newcammtx = calibdata['newcameramtx']

kernel = np.ones((5,5),np.uint8)

#colour ranges
color_ranges = {
    "red": ([0, 100, 100], [10, 255, 255]),
    "green": ([35, 50, 50], [85, 255, 255]),
    "blue": ([100, 50, 50], [140, 255, 255]),
    "yellow": ([20, 100, 100], [35, 255, 255])
}

#generating filter 
def generatefilter(image,colors):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = {}
    for color in colors:
        lower, upper = np.array(color_ranges[color][0]), np.array(color_ranges[color][1])
        masks[color] = cv2.inRange(hsv, lower, upper)
    return masks


class VideoPublisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(ImagePlusTupleList, '/rpi_video_feed', 1000)
        self.timer_ = self.create_timer(0.05, self.camera_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.bridge = CvBridge()
        self.colors = ["red", "green", "blue", "yellow"]

    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        h,w = frame.shape[:2]
        mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcammtx,(w,h),5)
        img = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
        color_masks = generatefilter(img, self.colors)

        for color, mask in color_masks.items():
           
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
           
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter contours by area
            min_area = 100 # Adjust this value as needed
            contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            
            self.get_logger().info(f'Number of {color} contours found: {len(contours)}')

            
            msg = ImagePlusTupleList()
            msg.image = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                self.get_logger().info(f'{color.capitalize()} contour area: {area}')
                contour_msg = Contour()
                contour_msg.color = color
                for point in cnt.reshape(-1, 2):
                    p = Point()
                    p.x = int(point[0])
                    p.y = int(point[1])
                    contour_msg.points.append(p)
                    msg.cnt.append(contour_msg)

        self.pub_.publish(msg)
        

       

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    print("Node started")
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
