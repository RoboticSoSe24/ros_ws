import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge

import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription

        # Create a publisher for the traffic_light topic
        self.traffic_light_pub = self.create_publisher(Bool, 'traffic_light', 1)

        self.drive = False


    def scanner_callback(self, data):

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # get image size
        height, width = img_cv.shape[:2]
        segment_height = height // 5
        segment_width = width // 3

        cropped_img = img_cv[2 * segment_height : 3 * segment_height, 2  * segment_width:]
        height, width, _ = cropped_img.shape
    
        B, G, R = cv2.split(cropped_img)

        total_rgb_img = np.zeros((height, width), dtype='uint16')
        total_rgb_img += B
        total_rgb_img += G
        total_rgb_img += R

        # Set pixels with total RGB value more than 600 to black
        thresh_upper = np.uint8(cv2.threshold(total_rgb_img, 600.0, 65535.0, cv2.THRESH_BINARY_INV)[1])
        # Set pixels with total RGB value less than 150 to black
        thresh_lower = np.uint8(cv2.threshold(total_rgb_img, 150.0, 65535.0, cv2.THRESH_BINARY)[1])
        # Set pixels to black if red to green difference is less than 80
        thresh_diff = np.uint8(cv2.threshold(np.abs(np.int16(G) - np.int16(R)), 80.0, 65535.0, cv2.THRESH_BINARY)[1])

        # mask out areas from the original image where thresholds didn't pass
        mask = thresh_upper & thresh_lower & thresh_diff
        cropped_img &= cv2.merge((mask, mask, mask))
        
        # compare number of red to green pixels
        total_red = cv2.sumElems(cropped_img[:,:,2])
        total_green = cv2.sumElems(cropped_img[:,:,1])

        if total_red > total_green:
            self.drive = False
            #self.get_logger().info('stoppen r = {}'.format(total_red))
        else:
            self.drive = True
            #self.get_logger().info('fahren r = {}'.format(total_red))

        # Publish to the traffic_light topic
        msg = Bool()
        msg.data = self.drive
        self.traffic_light_pub.publish(msg)

        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
