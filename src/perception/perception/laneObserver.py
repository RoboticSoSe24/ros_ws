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

        # declare parameters
        self.declare_parameter('canny_threshold', 50)
        self.declare_parameter('line_width', 16)
        self.declare_parameter('lane_width', 89)

        # create timer for updating parameter values
        self.parameterTimer = self.create_timer(3, self.parameter_callback)
        self.parameter_callback()

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

        # create publisher for topic 'lanes'
        self.lanePublisher_ = self.create_publisher(PoseArray, 'lanes', 10)

        self.get_logger().info('initialized laneObserver')


    def parameter_callback(self):
        # fetch parameters
        self.cannyThreshold = self.get_parameter('canny_threshold').get_parameter_value().double_value
        self.lineWidth      = self.get_parameter('line_width').get_parameter_value().integer_value
        self.laneWidth      = self.get_parameter('lane_width').get_parameter_value().integer_value
        self.get_logger().info('parameters updated')


    def image_callback(self, data):
        imSize = (320, 240)
        shape1C = (imSize[1], imSize[0], 1)
        shape3C = (imSize[1], imSize[0], 3)

        canvas1C = np.zeros(shape1C, dtype='uint8')
        canvas3C = np.zeros(shape3C, dtype='uint8')

        # retrieve camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        img_cv = cv2.resize(img_cv, imSize)

        # get lightness channel
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HLS)
        img_cv = cv2.split(img_cv)[1]
        cv2.imshow("source", img_cv)


        # pad into bigger matrix
        sub = np.zeros((shape1C[0]*4, shape1C[1]*4, 1), dtype = 'uint8')
        sub[720:960, 480:800, 0] = img_cv

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


        # find all lines with correct width 
        edges = cv2.Canny(warp, 60, 150)                                # get all edges
        dilate = cv2.dilate(edges, cv2.getStructuringElement(           # connect close enough edges to lines through dilation
            cv2.MORPH_ELLIPSE, (int(self.lineWidth/2), int(self.lineWidth/2))))  
        erode = cv2.erode(dilate, cv2.getStructuringElement(            # remove lines thinner than minimum width through erosion
            cv2.MORPH_ELLIPSE, (self.lineWidth-2,      self.lineWidth-2)))   
        canvas3C[:,:,0] = dilate
        canvas3C[:,:,1] = erode
        canvas3C[:,:,2] = edges
        cv2.imshow("dilation, erosion, edges", canvas3C)


        erode = cv2.dilate(erode, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (40, 40)))   # dilate to connect dotted line into one contour
        contours,_ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)        # get remaining contours
        if contours is None or len(contours) < 2:
            return
        
        # determine centerline as overlap between the two largest lines
        contours = sorted(contours, key=lambda cnt: cv2.contourArea(cnt), reverse=True)[:2]
        img1 = np.zeros(shape1C[:2], dtype='uint8')
        cv2.drawContours(img1, [contours[0]], 0, (255,255,255), self.laneWidth)     # dilate to achieve an overlap with the other line
        img2 = np.zeros(shape1C[:2], dtype='uint8')
        cv2.drawContours(img2, [contours[1]], 0, (255,255,255), self.laneWidth)     # dilate to achieve an overlap with the other line
        centerImg = cv2.ximgproc.thinning(img1 & img2, 0)                           # binary AND to get only overlap region
        cv2.rectangle(centerImg, (0,0), imSize, (0,0,0), 2)                         # mask out points on the edges of the image 
        canvas3C[:,:,0] = img1 * 0.5 + (img1 & img2) * 0.25 - centerImg
        canvas3C[:,:,1] = img2 * 0.5 + (img1 & img2) * 0.25 - centerImg
        canvas3C[:,:,2] = centerImg
        cv2.imshow("contour 1, contour 2, centerline", canvas3C)
        

        linesP = cv2.HoughLinesP(centerImg, 1, np.pi / 360, 30, None, 30, 5)    # get line as segments
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
        canvas3C = np.zeros(shape3C, dtype='uint8')
        for i in range(len(array)):
            pt1 = (array[i][0]+160, 240-array[i][1])
            cv2.circle(canvas3C, pt1, 2, (0,0,255))
            cv2.putText(canvas3C, str(i), pt1, cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,255,0))
            if i < len(array)-1:
                pt2 = (array[i+1][0]+160, 240-array[i+1][1])
                cv2.line(canvas3C, pt1, pt2, (0,255,0))
        cv2.imshow("final points", canvas3C)

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