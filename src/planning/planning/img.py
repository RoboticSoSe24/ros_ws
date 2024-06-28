import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
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


        # Create a directory to save images if it does not exist
        self.save_dir = "straight"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def scanner_callback(self, data):
        # Convert message to OpenCV image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # Crop the image to the region of interest
        cropped_img = img_cv[85:130, 250:290]

        # Save the image (optional)
        cv2.imwrite(os.path.join(self.save_dir, f"{self.get_clock().now().nanoseconds}.jpg"), cropped_img)

        # Display the original image and the cropped segment using OpenCV
        cv2.imshow("Original Image", img_cv)
        cv2.imshow("Cropped Segment", cropped_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
