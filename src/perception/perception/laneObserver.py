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
        self.declare_parameter('canny_threshold', 50.0)
        self.declare_parameter('line_width', 16)
        self.declare_parameter('lane_width', 130)
        self.declare_parameter('dot_line_length', 30)

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
        self.canny_threshold = self.get_parameter('canny_threshold').get_parameter_value().double_value
        self.line_width      = self.get_parameter('line_width').get_parameter_value().integer_value
        self.lane_width      = self.get_parameter('lane_width').get_parameter_value().integer_value
        self.dot_line_length = self.get_parameter('dot_line_length').get_parameter_value().integer_value
        self.get_logger().info('parameters updated')


    def image_callback(self, data):
        imSize = (320, 240)
        shape1C = (imSize[1], imSize[0], 1)
        shape3C = (imSize[1], imSize[0], 3)

        # retrieve camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        # get lightness channel
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HLS)
        img_cv = cv2.split(img_cv)[1]
        cv2.imshow("source", img_cv)

        # warp perspective
        warp = self.__perspective_transform(img_cv, imSize)

        # filter for line segments with the correct width
        filter = self.__filter_line_width(warp)

        # connect line segments into lines
        connect = self.__connect_lines(filter)


        # find all remaining contours
        contours,_ = cv2.findContours(connect, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or len(contours) < 2:
            cv2.waitKey(1)
            return

        # convert to array of tupels with number of pixels and pixel ratio        
        temp = []
        canvas3C = np.zeros(shape3C, dtype='uint8')
        for cnt in contours:
            img1 = np.zeros(shape1C[:2], dtype='uint8')
            cv2.drawContours(img1, [cnt], 0, (255,255,255), 6)
            pixelsConnected = np.sum(img1 == 255)

            img2 = cv2.dilate(filter, cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6)))
            img2 &= img1
            pixelsDotted = np.sum(img2 == 255)

            temp.append((cnt, pixelsConnected, pixelsDotted / pixelsConnected))
            
            canvas3C[:,:,0] += img1
            canvas3C[:,:,1] += img2

            x,y,w,h = cv2.boundingRect(cnt)
            cv2.putText(canvas3C, str(int(temp[-1][2] * 100)), 
                        (x+int(w/4),y+int(h/2)), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,0,255))
        
        contours = temp
        cv2.imshow("contour fill ratios", canvas3C)


        # keep 3 largest contours
        contours = sorted(contours, key=lambda cnt: cnt[1], reverse=True)[:3]
        while len(contours) < 3:
            contours.append(([(0,0)], 0, 0))


        img1 = np.zeros(shape1C[:2], dtype='uint8')
        cv2.drawContours(img1, [contours[0][0]], 0, (255,255,255), self.lane_width)     # dilate to achieve an overlap with the other line
        img2 = np.zeros(shape1C[:2], dtype='uint8')
        cv2.drawContours(img2, [contours[1][0]], 0, (255,255,255), self.lane_width)   # dilate to achieve an overlap with the other line
        centerImg = cv2.ximgproc.thinning(img1 & img2, 0)                           # binary AND to get only overlap region
        cv2.rectangle(centerImg, (0,0), imSize, (0,0,0), 2)                         # mask out points on the edges of the image 
        canvas3C[:,:,0] = img1 * 0.5 + (img1 & img2) * 0.25
        canvas3C[:,:,1] = img2 * 0.5 + (img1 & img2) * 0.25
        canvas3C[:,:,2] = centerImg
        cv2.imshow("contour 1, contour 2, centerline", canvas3C)


        # move dotted line to the front
        if (0.6 > contours[1][2]) and (contours[1][2] > 0.4):
            temp = contours[1]
            contours[1] = contours[0]
            contours[0] = temp
        elif (0.6 > contours[2][2]) and (contours[2][2] > 0.4):
            temp = contours[2]
            contours[2] = contours[1]
            contours[1] = contours[0]
            contours[0] = temp


        linesP = cv2.HoughLinesP(centerImg, 1, np.pi / 360, 30, None, 30, 5)    # get line as segments
        if linesP is None:
            cv2.waitKey(1)
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


    def __perspective_transform(self, img, out_size=(320, 240), debug_img=True):
        img = cv2.resize(img, (320, 240))

        # pad into bigger matrix
        sub = np.zeros((960, 1280, 1), dtype='uint8')
        sub[720:960, 480:800, 0] = img

        # perspective transform
        pts1 = np.float32([             # source points (measured prior)
            [104 + 480, 178 + 720], 
            [191 + 480, 178 + 720], 
            [67  + 480, 231 + 720], 
            [225 + 480, 233 + 720]])
        pts2 = np.float32([             # destination points
            [480, 640], 
            [800, 640], 
            [480, 960], 
            [800, 960]])
        M = cv2.getPerspectiveTransform(pts1,pts2)        # transformation matrix
        warp = cv2.warpPerspective(sub, M, (1280, 960))

        warp = cv2.resize(warp, out_size)
        warp = cv2.GaussianBlur(warp, (5, 5), 0)

        # show debug visuals
        if debug_img:
            cv2.imshow("warped", warp)
            cv2.waitKey(1)

        return warp


    def __filter_line_width(self, img, debug_img=True):
        # find edges
        edges = cv2.Canny(img, self.canny_threshold, self.canny_threshold * 3)

        # cover edges found near the corners by perspective transformation
        _,empty = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV)
        empty = cv2.dilate(empty, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
        edges &= ~empty   

        # connect close enough edges to lines through dilation
        dilate = cv2.dilate(edges, cv2.getStructuringElement(                   
            cv2.MORPH_ELLIPSE, (int(self.line_width/2), int(self.line_width/2))))  
        
        # prevent connections to the edges of the image
        cv2.rectangle(dilate, (0,0), (dilate.shape[1]-1, dilate.shape[0]-1), (0,0,0), 1)

        # remove lines thinner than minimum width by eroding from empty spots
        erode = cv2.erode(dilate, cv2.getStructuringElement(                    
            cv2.MORPH_ELLIPSE, (self.line_width-2, self.line_width-2)))   

        # remove lines thinner than minimum width by eroding from the edges        
        #dilate &= ~edges
        #erode = cv2.erode(dilate, cv2.getStructuringElement(
        #    cv2.MORPH_ELLIPSE, (int(self.line_width/2)-2, int(self.line_width/2)-2)))

        # filter out point like artifacts
        filter = cv2.boxFilter(erode, -1, (5, 5))
        _,filter = cv2.threshold(filter, 50, 255, cv2.THRESH_BINARY)

        # show debug visuals
        if debug_img:
            canvas = np.zeros((edges.shape[0], edges.shape[1], 3), dtype='uint8')
            canvas[:,:,0] = dilate
            canvas[:,:,1] = filter
            canvas[:,:,2] = edges
            cv2.imshow("line width filter", canvas)
            cv2.waitKey(1)

        return filter


    def __connect_lines(self, img, debug_img=True):
        # get current line segments as contours
        contours,_ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None:
            return np.zeros(img.shape, dtype='uint8')
        
        # extend contours along their main axis
        connect = img.copy()
        for cnt in contours:
            top = max(cnt, key=lambda pt: pt[0][1])[0]
            bot = min(cnt, key=lambda pt: pt[0][1])[0]

            if (top[1] == bot[1]) or (np.linalg.norm(top - bot) > self.dot_line_length + 5):
                continue
            vec = (top - bot) / np.linalg.norm(top - bot) * self.dot_line_length / 2
            vec = np.int0(vec)
            
            cv2.line(connect, top+vec, bot-vec, (255,255,255), 1)

        # dilate to connect dotted line into one contour
        connect = cv2.dilate(connect, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.dot_line_length, self.dot_line_length)))     
        
        # avoid shrinking artifacts on the edges
        cv2.rectangle(connect, (0, 0), (connect.shape[1]-1, connect.shape[0]-1), (0,0,0), 1)    
        
        # shrink lines back down to width of 1
        connect = cv2.ximgproc.thinning(connect, 0)     

        # show debug visuals
        if debug_img:
            cv2.imshow("connected lines", connect)
            cv2.waitKey(1)
        
        return connect



def main(args=None):
    rclpy.init(args=args)

    lane_observer = LaneObserver()

    rclpy.spin(lane_observer)
    
    lane_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()