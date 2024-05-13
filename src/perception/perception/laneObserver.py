import rclpy
from rclpy.node import Node

from interfaces.msg import Lanes
from geometry_msgs.msg import Point

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class LaneObserver(Node):

    def __init__(self):
        super().__init__('lane_observer')

        # declare parameters
        self.declare_parameter('canny_threshold', 40.0)
        self.declare_parameter('line_width', 8)
        self.declare_parameter('line_tolerance', 4)
        self.declare_parameter('lane_width', 130)
        self.declare_parameter('dot_line_length', 20)
        self.declare_parameter('dot_line_tolerance', 5)

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
        self.lanePublisher_ = self.create_publisher(Lanes, 'lanes', 10)

        self.get_logger().info('initialized laneObserver')


    def parameter_callback(self):
        # fetch parameters
        self.canny_threshold    = self.get_parameter('canny_threshold').get_parameter_value().double_value
        self.line_width         = self.get_parameter('line_width').get_parameter_value().integer_value
        self.line_tolerance     = self.get_parameter('line_tolerance').get_parameter_value().integer_value
        self.lane_width         = self.get_parameter('lane_width').get_parameter_value().integer_value
        self.dot_line_length    = self.get_parameter('dot_line_length').get_parameter_value().integer_value
        self.dot_line_tolerance = self.get_parameter('dot_line_tolerance').get_parameter_value().integer_value
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

        # undistort image
        img_cv = self.__undistort(img_cv)

        # warp perspective
        warp = self.__perspective_transform(img_cv, imSize)

        # find line intersections hinting at parking spots
        self.__find_intersections(warp)

        # filter for line segments with the correct width
        filter = self.__filter_line_width(warp)

        # try to find the middle line
        found_dotted, dotted = self.__find_dotted_line(filter)

        if found_dotted:
            dotted = cv2.dilate(dotted, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.lane_width, self.lane_width)))
            lanesImg = cv2.dilate(dotted, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) & ~dotted
            #cv2.imshow("left and right lanes", lanesImg)
            #cv2.waitKey(1)

        else:
            # connect line segments into lines
            connect = self.__connect_lines(filter, False)

            # find all remaining contours
            contours,_ = cv2.findContours(connect, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours is None or len(contours) < 2:
                return

            # convert to array of tupels with number of pixels and pixel ratio        
            temp = []
            for cnt in contours:
                img1 = np.zeros(shape1C[:2], dtype='uint8')
                cv2.drawContours(img1, [cnt], 0, (255,255,255), 6)
                pixelsConnected = np.sum(img1 == 255)

                img2 = cv2.dilate(filter, cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6)))
                img2 &= img1
                pixelsDotted = np.sum(img2 == 255)

                temp.append((cnt, pixelsConnected, pixelsDotted / pixelsConnected))
                
            contours = sorted(temp, key=lambda cnt: cnt[1], reverse=True)[:2]

            # get centerline between two largest contours
            img1 = np.zeros(shape1C[:2], dtype='uint8')
            cv2.drawContours(img1, [contours[0][0]], 0, (255,255,255), self.lane_width) # dilate to achieve an overlap with the other line
            img2 = np.zeros(shape1C[:2], dtype='uint8')
            cv2.drawContours(img2, [contours[1][0]], 0, (255,255,255), self.lane_width) # dilate to achieve an overlap with the other line
            centerImg = cv2.ximgproc.thinning(img1 & img2, 0)                           # binary AND to get only overlap region
            cv2.rectangle(centerImg, (0,0), imSize, (0,0,0), 2)                         # mask out points on the edges of the image 
            canvas3C = np.zeros(shape3C, dtype='uint8')
            canvas3C[:,:,0] = img1 * 0.5 + (img1 & img2) * 0.25
            canvas3C[:,:,1] = img2 * 0.5 + (img1 & img2) * 0.25
            canvas3C[:,:,2] = centerImg
            cv2.imshow("contour 1, contour 2, centerline", canvas3C)
            cv2.waitKey(1)

            # get line segments from pixels
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

            # publish points to 'lanes' topic
            lanes = Lanes()
            for pt in array:
                point = Point()
                point.x = float(pt[0])
                point.y = float(pt[1])
                lanes.right.append(point)
            self.lanePublisher_.publish(lanes)


    def __undistort(self, img, debug_img=True):
        img = cv2.resize(img, (640, 480))
        self.camera_matrix = np.array( [[426.583512, 0.000000,   308.964505],
                                        [0.000000,   814.271722, 223.209438],
                                        [0.0,        0.0,        1.0]])        
        self.camera_distortion = np.array([[-0.477731, 0.162638, 0.005566, -0.003685, 0.000000]])
        self.camera_optimal_matrix, self.camera_optimal_roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, 
                                                                                            self.camera_distortion, 
                                                                                            (640, 480), 1)

        img = cv2.undistort(img, self.camera_matrix, self.camera_distortion, None, self.camera_optimal_matrix)

        # show debug visuals
        if debug_img:
            cv2.imshow("undistorted", img)
            cv2.waitKey(1)

        return img


    def __perspective_transform(self, img, out_size=(320, 240), debug_img=True):
        r = img.shape[0]
        c = img.shape[1]
        sub = np.zeros((4*r, 4*c), dtype='uint8')
        sub[3*r:4*r, int(1.5*c):int(2.5*c)] = img

        k = c / 640
        pts1 = np.float32([             # undistorted source points
            [k*231 + 1.5*c, k*324 + 3*r], 
            [k*350 + 1.5*c, k*322 + 3*r], 
            [k*161 + 1.5*c, k*426 + 3*r], 
            [k*412 + 1.5*c, k*419 + 3*r]])
        pts2 = np.float32([             # destination points
            [1.5*c, 4*r-c], 
            [2.5*c, 4*r-c], 
            [1.5*c, 4*r], 
            [2.5*c, 4*r]])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        warp = cv2.warpPerspective(sub, M, (4*c, 4*r))

        warp = cv2.resize(warp, out_size)
        warp = cv2.GaussianBlur(warp, (5, 5), 0)

        # show debug visuals
        if debug_img:
            cv2.imshow("warped", warp)
            cv2.waitKey(1)

        return warp


    def __find_intersections(self, img, debug_img=True):
        filter = np.zeros((30, 30), dtype='float32')
        cv2.line(filter, (14, 0), (14, 29), (1.0, 1.0, 1.0), self.line_width)
        cv2.line(filter, (14, 14), (29, 14), (1.0, 1.0, 1.0), self.line_width)
        cv2.imshow('filter', filter)
        #filter /= np.sum(filter)
        filter -= np.ones(filter.shape, dtype='float32') * 0.5

        img = np.float32(img)

        filtered = cv2.filter2D(img, 1, filter)
        cv2.imshow('filtered', filtered / 255.0)


    def __filter_line_width(self, img, debug_img=True):
        # find edges
        edges = cv2.Canny(img, self.canny_threshold, self.canny_threshold * 3)

        # cover edges found near the corners by perspective transformation
        _,empty = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV)
        empty = cv2.dilate(empty, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
        edges &= ~empty   

        # connect close enough edges to lines through dilation
        dilate = cv2.dilate(edges, cv2.getStructuringElement(                   
            cv2.MORPH_ELLIPSE, (self.line_width, self.line_width)))  
        
        # prevent connections to the edges of the image
        cv2.rectangle(dilate, (0,0), (dilate.shape[1]-1, dilate.shape[0]-1), (0,0,0), 1)


        # remove lines thinner than minimum width through multiple erosion steps
        # 1. erode original dilation
        erode = cv2.erode(dilate, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.line_width, self.line_width)))
        # 2. remove edges
        erode &= ~edges
        # 3. erode further to remove lines thinner than desired
        erode = cv2.erode(erode, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.line_width - self.line_tolerance, self.line_width - self.line_tolerance)))

        # remove very small leftovers
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


    def __find_dotted_line(self, img, debug_img=True):
        # get all contours in the image
        contours,_ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None:
            return False, None

        # find promising line segments among contours
        canvas = np.zeros((img.shape[0], img.shape[1], 3), dtype='uint8')
        segments = []
        for cnt in contours:
            ptMax = max(cnt, key=lambda pt: pt[0][1])[0]
            ptMin = min(cnt, key=lambda pt: pt[0][1])[0]
            d = np.linalg.norm(ptMin - ptMax)
            if d > self.dot_line_length - self.dot_line_tolerance and d < self.dot_line_length + self.dot_line_tolerance:
                cv2.drawContours(canvas, [cnt], 0, (0,0,255))
                cv2.putText(canvas, str(d), ptMax, cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,255,255))
                segments.append((ptMin, ptMax, (ptMin - ptMax) / d))
                cv2.line(canvas, ptMin, np.int32(ptMin + segments[-1][2] * 10), (0, 255, 0))

        if len(segments) < 1:
            return False, None

        segments.sort(key=lambda seg: seg[0][1], reverse=True)          # sort bottom up

        # starting from each line segment try to form one single line
        lines = []
        for i in range(len(segments)):
            # starting segment should be in the bottom half of the image
            if segments[i][0][1] < img.shape[0] / 2:
                continue
            line = [segments[i]]
            for seg in segments[i+1:]:
                # add on line segments with a minimum distance and deviation from the previous segment
                dev = np.linalg.norm(np.cross(line[-1][1]-line[-1][0], line[-1][0]-seg[1])) / np.linalg.norm(line[-1][1]-line[-1][0])
                dot = np.dot(line[-1][2], seg[2])
                dist = np.linalg.norm(line[-1][0] - seg[1])
                if dev < self.dot_line_length and dot > 0.90 and dist > self.dot_line_length:
                    line.append(seg)
            lines.append(line)

        if len(lines) == 0:
            return False, None

        # finally select line that most segments were assigned to
        lines.sort(key=lambda line: len(line), reverse=True)
        line = lines[0]

        # draw final line
        lineImg = np.zeros(img.shape[:2], dtype='uint8')
        for i in range(len(line)):
            seg = line[i]
            cv2.line(lineImg, seg[0], seg[1], (255,255,255))    # draw segment
            if i == 0:                                          # extend first segment out the bottom
                cv2.line(lineImg, seg[1], np.int32(seg[1] - seg[2] * 1000), (255,255,255))
            if i > 0:                                           # connect with previous segment
                cv2.line(lineImg, seg[1], line[i-1][0], (255,255,255))
            if i == len(line)-1:                                # extent last segment out the top
                cv2.line(lineImg, seg[0], np.int32(seg[1] + seg[2] * 1000), (255,255,255))

        # show debug visuals
        if debug_img:
            canvas[:,:,1] = lineImg
            cv2.imshow("dotted line", canvas)
            cv2.waitKey(1)

        return True, lineImg


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