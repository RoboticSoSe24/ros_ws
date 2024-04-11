import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import cv2
import numpy as np
import random as rng

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class LaneObserver(Node):

    def __init__(self):
        super().__init__('lane_observer')

        # create publisher for topic 'lanes'
        self.lanePublisher_ = self.create_publisher(PoseArray, 'lanes', 10)

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
        imSize = (320, 240)
        shape1C = (imSize[1], imSize[0], 1)
        shape3C = (imSize[1], imSize[0], 3)

        # retrieve camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        img_cv = cv2.resize(img_cv, imSize)

        # get lightness channel
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HLS)
        img_cv = cv2.split(img_cv)[1]

        # pad into bigger matrix
        sub = np.zeros((shape1C[0]*4, shape1C[1]*4, 1), dtype = "uint8")
        sub[720:960, 480:800, 0] = img_cv
        cv2.imshow("padded src", sub)

        # perspective transform
        pts1 = np.float32([             # source points (measured prior)
            [104 + 480, 178 + 720], 
            [191 + 480, 178 + 720], 
            [67  + 480, 231 + 720], 
            [225 + 480, 233 + 720]])
        pts2 = np.float32([             # destination points
            [480,640], 
            [800,640], 
            [480,960], 
            [800,960]])
        M = cv2.getPerspectiveTransform(pts1,pts2)        # transformation matrix
        warp = cv2.warpPerspective(sub, M, (imSize[0]*4, imSize[1]*4))

        warp = cv2.resize(warp, imSize)
        warp = cv2.GaussianBlur(warp, (5, 5), 0)
        cv2.imshow("warped", warp)

        # find edges
        edges = cv2.Canny(warp, 40, 150)
        cv2.imshow("canny", edges)

        # connect edges to lines through dilation
        dilate = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10)))

        # remove lines thinner than minimum width through erosion
        erode = cv2.erode(dilate, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (16, 16)))
        cv2.imshow("dilate_erode", erode)

        # dilate to connect dotted line into one contour
        erode = cv2.dilate(erode, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (40, 40)))   

        # get remaining contours
        contours,_ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or len(contours) < 2:
            return
        
        # determine centerline as overlap between the two largest lines
        contours = sorted(contours, key=lambda cnt: cv2.contourArea(cnt), reverse=True)[:2]
        img1 = np.zeros(shape1C, dtype="uint8")
        cv2.drawContours(img1, [contours[0]], 0, (255,255,255), 87)     # dilate to achieve an overlap with the other line
        img2 = np.zeros(shape1C, dtype="uint8")
        cv2.drawContours(img2, [contours[1]], 0, (255,255,255), 87)     # dilate to achieve an overlap with the other line
        centerImg = cv2.ximgproc.thinning(img1 & img2, 0)               # binary AND to get only overlap region
        cv2.imshow("centerline", centerImg)
        
        # get line as segments
        linesP = cv2.HoughLinesP(centerImg, 1, np.pi / 360, 30, None, 30, 5)
        if linesP is None:
            return
        
        # collect line points into array
        array = []
        for line in linesP:
            l = line[0]
            array.append((l[0]-160, 240-l[1]))
            array.append((l[2]-160, 240-l[3]))

        array.sort(key=lambda pt: np.linalg.norm(pt))

        # draw final points to image
        final = np.zeros(shape3C, dtype="uint8")
        for i in range(len(array)):
            pt1 = (array[i][0]+160, 240-array[i][1])
            cv2.circle(final, pt1, 2, (0,0,255))
            cv2.putText(final, str(i), pt1, cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,255,0))
            if i < len(array)-1:
                pt2 = (array[i+1][0]+160, 240-array[i+1][1])
                cv2.line(final, pt1, pt2, (0,255,0))
        cv2.imshow("final points", final)

        # publish points to 'lanes'
        poseArr = PoseArray()
        for pt in array:
            pose = Pose()
            pose.position.x = float(pt[0])
            pose.position.y = float(pt[1])
            poseArr.poses.append(pose)
        self.lanePublisher_.publish(poseArr)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    lane_observer = LaneObserver()

    rclpy.spin(lane_observer)
    
    lane_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()