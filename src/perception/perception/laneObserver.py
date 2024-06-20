import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from interfaces.msg import Lanes
from geometry_msgs.msg import Point

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError



class TrackbarParameter():

    def __init__(self, node, name, window_name, val, max_val):
        self.node = node
        self.name = name
        self.window_name = window_name
        self.val = val
        self.max_val = max_val

        node.declare_parameter(name, val)
        node.add_on_set_parameters_callback(self.on_parameters)

        cv2.createTrackbar(name, window_name, int(val), int(max_val), self.on_trackbar)

    def on_trackbar(self, val):
        self.val = val

    def on_parameters(self, params):
        for param in params:
            if param.name == self.name:
                self.val = param.value
                cv2.setTrackbarPos(self.name, self.window_name, int(self.val)) 
        return SetParametersResult(successful=True)

    def __int__(self):
        return int(self.val)
    
    def __float__(self):
        return float(self.val)



class LaneObserver(Node):

    def __init__(self):
        super().__init__('lane_observer')

        # declare relevant windows
        cv2.namedWindow("line width filter")
        cv2.moveWindow("line width filter", 1920, 0)
        cv2.namedWindow("connected lines")
        cv2.moveWindow("connected lines", 1920 + 400, 0)
        cv2.namedWindow("contour 1, contour 2, centerline")
        cv2.moveWindow("contour 1, contour 2, centerline", 1920 + 800, 0)

        # declare parameters
        self.canny_threshold    = TrackbarParameter(self, 'canny_threshold',    "line width filter",                50.0, 255.0)
        self.line_width         = TrackbarParameter(self, 'line_width',         "line width filter",                9,    20)
        self.line_tolerance     = TrackbarParameter(self, 'line_tolerance',     "line width filter",                4,    10)
        self.lane_width         = TrackbarParameter(self, 'lane_width',         "contour 1, contour 2, centerline", 170,  350)
        self.line_begin         = TrackbarParameter(self, 'line_begin',         "contour 1, contour 2, centerline", 200,  240)
        self.dot_line_length    = TrackbarParameter(self, 'dot_line_length',    "connected lines",                  25,   40)
        self.dot_line_tolerance = TrackbarParameter(self, 'dot_line_tolerance', "connected lines",                  5,    10)

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


    def image_callback(self, data):
        imSize = (320, 240)
        shape1C = (imSize[1], imSize[0], 1)
        shape3C = (imSize[1], imSize[0], 3)

        # retrieve camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        # get lightness channel
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HLS)
        img_cv = cv2.split(img_cv)[1]
        #cv2.imshow("source", img_cv)

        # undistort image
        img_cv = self.__undistort(img_cv, False)

        # warp perspective
        warp = self.__perspective_transform(img_cv, imSize, False)

        # filter for line segments with the correct width
        filter = self.__filter_line_width(warp)

        # connect line segments to lines
        connect = self.__connect_lines(filter)

        # find all remaining contours
        contours,_ = cv2.findContours(connect, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if max(cnt, key=lambda pt: pt[0][1])[0][1] > int(self.line_begin)]

        if contours is None or len(contours) < 2:
            return
        
        # keep two largest contours
        contours = sorted(contours, 
                          key=lambda cnt: np.linalg.norm(max(cnt, key=lambda pt: pt[0][1])[0] - min(cnt, key=lambda pt: pt[0][1])[0]),
                          reverse=True)[:2]

        # get centerline between two largest contours
        img1 = np.zeros(shape1C[:2], dtype='uint8')
        cv2.drawContours(img1, [contours[0]], 0, (255,255,255), int(self.lane_width)) # dilate to achieve an overlap with the other line
        img2 = np.zeros(shape1C[:2], dtype='uint8')
        cv2.drawContours(img2, [contours[1]], 0, (255,255,255), int(self.lane_width)) # dilate to achieve an overlap with the other line
        centerImg = cv2.ximgproc.thinning(img1 & img2, 0)                             # binary AND to get only overlap region
        cv2.rectangle(centerImg, (0,0), imSize, (0,0,0), 2)                           # mask out points on the edges of the image 
        canvas3C = np.zeros(shape3C, dtype='uint8')
        canvas3C[:,:,0] = img1 * 0.5 + (img1 & img2) * 0.25
        canvas3C[:,:,1] = img2 * 0.5 + (img1 & img2) * 0.25
        canvas3C[:,:,2] = centerImg
        cv2.line(canvas3C, (0, int(self.line_begin)), (320, int(self.line_begin)), (0,0,255))
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
            [k*228 + 1.5*c, k*324 + 3*r], 
            [k*353 + 1.5*c, k*322 + 3*r], 
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


    def __filter_line_width(self, img, debug_img=True):
        # find edges
        edges = cv2.Canny(img, float(self.canny_threshold), float(self.canny_threshold) * 3.0)

        # cover edges found near the corners by perspective transformation
        _,empty = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV)
        empty = cv2.dilate(empty, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
        edges &= ~empty   

        # connect close enough edges to lines through dilation
        dilate = cv2.dilate(edges, cv2.getStructuringElement(                   
            cv2.MORPH_ELLIPSE, (int(self.line_width), int(self.line_width))))  
        
        # prevent connections to the edges of the image
        cv2.rectangle(dilate, (0,0), (dilate.shape[1]-1, dilate.shape[0]-1), (0,0,0), 1)


        # remove lines thinner than minimum width through multiple erosion steps
        # 1. erode original dilation
        erode = cv2.erode(dilate, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (int(self.line_width), int(self.line_width))))
        # 2. remove edges
        erode &= ~edges
        # 3. erode further to remove lines thinner than desired
        erode = cv2.erode(erode, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (int(self.line_width) - int(self.line_tolerance), int(self.line_width) - int(self.line_tolerance))))

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


    def __connect_lines(self, img, debug_img=True):
        # get current line segments as contours
        contours,_ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None:
            return np.zeros(img.shape, dtype='uint8')
        
        # extend contours along their main axis
        connect = np.zeros(img.shape, dtype='uint8')
        for cnt in contours:
            ptMax = max(cnt, key=lambda pt: pt[0][1])[0]
            ptMin = min(cnt, key=lambda pt: pt[0][1])[0]

            d = np.linalg.norm(ptMin - ptMax)
            
            if int(self.dot_line_length) - int(self.dot_line_tolerance) < d < int(self.dot_line_length) + int(self.dot_line_tolerance):
                vec = np.int0((ptMax - ptMin) / d * int(self.dot_line_length))
                cv2.line(connect, ptMax+vec, ptMin-vec, (255,255,255), 5)
            
            if int(self.dot_line_length) + int(self.dot_line_tolerance) < d:
                cv2.drawContours(connect, [cnt], 0, (255,255,255), 5)

        # dilate to connect dotted line into one contour
        connect = cv2.dilate(connect, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (int(self.dot_line_tolerance), int(self.dot_line_tolerance))))     
        
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