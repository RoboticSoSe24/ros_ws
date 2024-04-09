import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
import numpy as np

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
        
        # get light channel
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HLS)
        img_cv = cv2.split(img_cv)[1]

        # pad into bigger matrix
        sub = np.zeros((960, 1280, 1), dtype = "uint8")
        sub[720:960, 480:800, 0] = img_cv

        pts1 = np.float32([
            [105 + 480, 178 + 720], 
            [190 + 480, 178 + 720], 
            [67  + 480, 231 + 720], 
            [225 + 480, 233 + 720]])
        
        pts2 = np.float32([
            [480,720], 
            [800,720], 
            [480,img_cv.shape[0]+720], 
            [img_cv.shape[1]+480,img_cv.shape[0]+720]])
        
        # perspective transform
        M = cv2.getPerspectiveTransform(pts1,pts2)        
        dst = cv2.warpPerspective(sub, M, (1280, 960))


        dst = cv2.resize(dst, None, fx = 0.5, fy = 0.5)
        ret, thresh = cv2.threshold(dst, 120, 255, cv2.THRESH_BINARY)

        # find line segments in threshold image
        linesP = cv2.HoughLinesP(thresh, 1, np.pi / 180.0, 50, None, 30, 10)
        cdstP = np.zeros((480, 640, 3), dtype = "uint8")
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        

        cv2.imshow("img", sub)
        cv2.imshow("warped", dst)
        cv2.imshow("thresh", thresh)
        cv2.imshow("line segments", cdstP)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    lane_observer = LaneObserver()

    rclpy.spin(lane_observer)
    
    lane_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()