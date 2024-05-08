import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import Bool


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
        self.traffic_light_pub = self.create_publisher(Bool, 'traffic_light', qos_profile=qos_policy)

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


        cv2.imshow("Original Image", img_cv)

        # Set pixels with total RGB value less than 150 and more than 600 to black
        # Set pixels to black if differenz is less than 80
        #Bildbearbeitung
        for y in range(height):
            for x in range(width):
                pixel = cropped_img[y, x]
                total_rgb = sum(pixel)
                if total_rgb > 600 or abs(int(pixel[1]) - int(pixel[2])) < 80 or total_rgb < 150:
                    cropped_img[y, x] = [0, 0, 0]  # Set pixel to black


        total_red = 0
        total_green = 0
        total_blue = 0

        for y in range(height):
            for x in range(width):
                pixel = cropped_img[y, x]
                total_blue += pixel[0]
                total_green += pixel[1]
                total_red += pixel[2]


        #Gesamtsumme der RGB-Werte:
        #print("Gesamtsumme der RGB-Werte:")
        #print("Rot:", total_red)
        #print("Grün:", total_green)
        #print("Blau:", total_blue)


        if total_red > total_green:
            self.drive = False
            #self.get_logger().info('stoppen r = {}'.format(total_red))
        else:
            self.drive = True
            #self.get_logger().info('fahren r = {}'.format(total_red))


        # Publish the traffic_light topic
        msg = Bool()
        msg.data = self.drive
        self.traffic_light_pub.publish(msg)

        # original image and modified image using OpenCV

        cv2.imshow("Modified Image", cropped_img)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
