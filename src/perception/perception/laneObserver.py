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
        # retrieve and display camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv2.imshow("img", img_cv)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    lane_observer = LaneObserver()

    rclpy.spin(lane_observer)
    
    lane_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()