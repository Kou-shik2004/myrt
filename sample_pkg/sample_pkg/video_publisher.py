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
    "red": ([0, 100, 100], [15, 255, 255]),
    "green": ([40, 70, 50], [90, 255, 255]),
    "blue": ([90, 71, 50], [130, 255, 255]),
    "yellow": ([20, 100, 100], [30, 255, 255]),
    "black": ([0, 0, 0], [180, 255, 30])
}

#generating filter 
def generatefilter(image,colour):
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower,upper = np.array(color_ranges[colour][0]),np.array(color_ranges[colour][1])
    filterimg = cv2.inRange(hsv,lower,upper)
    return filterimg


class VideoPublisher(Node):

    def __init__(self):
        super().__init__('rpi_video_publisher')
        self.pub_ = self.create_publisher(ImagePlusTupleList, '/rpi_video_feed', 100)
        self.timer_ = self.create_timer(0.05, self.camera_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.bridge = CvBridge()

    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        h,w = frame.shape[:2]
        mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcammtx,(w,h),5)
        img = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
        filter = generatefilter(img,"red")
        erode = cv2.erode(filter,kernel,iterations=2)
        edges = cv2.Canny(erode,50,150,apertureSize=3)
        contours,_ = cv2.findContours(erode,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f'Number of contours found: {len(contours)}')

        msg = ImagePlusTupleList()
        msg.image = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
    
        for cnt in contours:
            area = cv2.contourArea(cnt)
            self.get_logger().info(f'Contour area: {area}')
            contour_msg = Contour()
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
