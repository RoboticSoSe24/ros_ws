import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32

import cv2
from cv_bridge import CvBridge

import tensorflow as tf
import numpy as np

# Laden des Modells
model = tf.keras.models.load_model('./models/model_0.keras')


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        # declare parameters
        self.declare_parameter('min_sign_count', 10)

        self.bridge = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy
        )

        # Pub for Traffic-Light-Topic
        self.traffic_light_pub = self.create_publisher(Bool, 'traffic_light', 1)

        # Pub for signs-Topic
        self.sign_pub = self.create_publisher(Int32, 'signs', 1)
        self.sign = 2
        self.sign_count = 0


    def image_callback(self, data):
        # retrieve camera image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        img_cv = cv2.resize(img_cv, (320, 240))     # constant resize for later cropping

        # crop out relevant part of the image
        cropped_img = img_cv[85:130, 250:290]

        # check cropped image for traffic lights and signs
        self.__find_traffic_light(cropped_img.copy())
        self.__find_traffic_sign(img_cv)


    def __find_traffic_light(self, cropped_img):
        height, width, _ = cropped_img.shape

        B, G, R = cv2.split(cropped_img)

        total_rgb_img = np.zeros((height, width), dtype='uint16')
        total_rgb_img += B
        total_rgb_img += G
        total_rgb_img += R

        # Set pixels with a total RGB value above 600 to black
        thresh_upper = np.uint8(cv2.threshold(total_rgb_img, 600.0, 65535.0, cv2.THRESH_BINARY_INV)[1])
        # Set pixels with a total RGB value below 150 to black
        thresh_lower = np.uint8(cv2.threshold(total_rgb_img, 200.0, 65535.0, cv2.THRESH_BINARY)[1])
        # Set pixels to black if the difference between red and green is less than 80
        thresh_diff = np.uint8(cv2.threshold(np.abs(np.int16(G) - np.int16(R)), 80.0, 65535.0, cv2.THRESH_BINARY)[1])

        # apply mask
        mask = thresh_upper & thresh_lower & thresh_diff
        cropped_img &= cv2.merge((mask, mask, mask))

        cv2.imshow("traffic_light", cropped_img)
        cv2.waitKey(1)

        # Comparison of the number of red and green pixels
        total_red = cv2.sumElems(cropped_img[:, :, 2])
        total_green = cv2.sumElems(cropped_img[:, :, 1])

        #self.get_logger().info('total_red = {})'.format(total_red))
        #self.get_logger().info('total_green = {})'.format(total_green))

        # Publish message
        msg = Bool()
        msg.data = False if (total_red > total_green) else True
        self.traffic_light_pub.publish(msg)


    def __find_traffic_sign(self, img_cv):
        min_sign_count = self.get_parameter('min_sign_count').get_parameter_value().integer_value

        # cut out multiple images to pass to CNN
        input_images = np.zeros((3, 50, 42))
        for i in range(3):
            crop = img_cv[85 : 130, 250+i*8 : 290+i*8]          # cut out at slight offset
            crop = cv2.resize(crop, (42, 50))                   # adjust size
            crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)       # convert to grayscale
            crop = np.float32(np.array(crop)) / 255             # normalize pixel values
            input_images[i,:,:] = crop

        cv2.imshow("sign", crop)
        cv2.waitKey(1)

        # find most common prediction among input images
        prediction = model.predict(input_images, verbose=0)
        pred_sum = np.zeros(5)
        for p in prediction:
            pred_sum += p
        pred_class = np.argmax(pred_sum)

        # count consecutive occurrences of sign class
        if pred_class != self.sign:
            self.sign_count = 0
            self.sign = pred_class
        self.sign_count += 1

        # publish sign
        msg = Int32()
        msg.data = int(pred_class if self.sign_count > min_sign_count else 2)
        self.sign_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
