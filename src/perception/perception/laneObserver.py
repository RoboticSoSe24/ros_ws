import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
import numpy as np
import random as rng

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class LaneObserver(Node):

    def __init__(self):
        super().__init__('lane_observer')

        # create publisher for 'topic'
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # create timer for 'topic' publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data over WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscriber for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def image_callback(self, data):
        # retrieve camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        img_cv = cv2.resize(img_cv, (320, 240))

        # get lightness channel
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HLS)
        img_cv = cv2.split(img_cv)[1]

        # pad into bigger matrix
        sub = np.zeros((960, 1280, 1), dtype = "uint8")
        sub[720:960, 480:800, 0] = img_cv
        cv2.imshow("padded src", sub)

        # perspective transform
        pts1 = np.float32([             # source points (measured prior)
            [104 + 480, 178 + 720], 
            [191 + 480, 178 + 720], 
            [67  + 480, 231 + 720], 
            [225 + 480, 233 + 720]])
        pts2 = np.float32([             # destination points
            [480,720], 
            [800,720], 
            [480,img_cv.shape[0]+720], 
            [img_cv.shape[1]+480,img_cv.shape[0]+720]])
        M = cv2.getPerspectiveTransform(pts1,pts2)        # transformation matrix
        dst = cv2.warpPerspective(sub, M, (1280, 960))

        dst = cv2.resize(dst, None, fx = 0.25, fy = 0.25)
        dst = cv2.GaussianBlur(dst, (5, 5), 0)
        cv2.imshow("warped", dst)


        # find edges
        edges = cv2.Canny(dst, 40, 150)
        cv2.imshow("canny", edges)


        # find lines thicker than minimum width
        dilate = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10)))
        erode = cv2.erode(dilate, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (16, 16)))
        cv2.imshow("dilate_erode", erode)


        # find centerline between the two largest lines in the image
        erode = cv2.dilate(erode, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (40, 40)))   # dilate to connect dotted line into one contour
        contours,_ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or len(contours) < 2:
            return
        
        contours = sorted(contours, key=lambda cnt: cv2.contourArea(cnt), reverse=True)[:2]
        img1 = np.zeros(erode.shape, dtype="uint8")
        cv2.drawContours(img1, [contours[0]], 0, (255,255,255), 87)     # dilate to provoce an overlap with the other line
        img2 = np.zeros(erode.shape, dtype="uint8")
        cv2.drawContours(img2, [contours[1]], 0, (255,255,255), 87)     # dilate to achieve an overlap with the other line
        centerImg = cv2.ximgproc.thinning(img1 & img2, 0)               # binary AND to get only overlap region
        cv2.imshow("centerline", centerImg)
        
        linesP = cv2.HoughLinesP(centerImg, 1, np.pi / 360, 30, None, 30, 5)
        cdstP = np.zeros((240, 320, 3), dtype="uint8")
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                cv2.circle(cdstP, (l[0], l[1]), 2, (0,255,0))
                cv2.circle(cdstP, (l[2], l[3]), 2, (0,255,0))
        cv2.imshow("centerline hough", cdstP)


        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    lane_observer = LaneObserver()

    rclpy.spin(lane_observer)
    
    lane_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()