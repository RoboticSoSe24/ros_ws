import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32

import cv2
from cv_bridge import CvBridge

import tensorflow as tf
import numpy as np

# Laden des Modells
model = tf.keras.models.load_model('./models/model2.h5')


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

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


        self.drive = False

    def image_callback(self, data):
        # Convert message to OpenCV image and crop it
        img_cv, cropped_img = self.process_image(data)

        # Call method
        self.traffic_light_processing(cropped_img.copy())
        self.street_sign_processing(cropped_img)

    def process_image(self, data):
        # Convert to OpenCV-Img
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # Img size:
        height, width = img_cv.shape[:2]
        segment_height = height // 5
        segment_width = width // 8

        # cut img
        cropped_img = img_cv[2 * segment_height: 3 * segment_height, 7 * segment_width:]

        return img_cv, cropped_img

    def traffic_light_processing(self, cropped_img):
        height, width, _ = cropped_img.shape

        B, G, R = cv2.split(cropped_img)

        total_rgb_img = np.zeros((height, width), dtype='uint16')
        total_rgb_img += B
        total_rgb_img += G
        total_rgb_img += R

        # Set pixels with a total RGB value above 600 to black
        thresh_upper = np.uint8(cv2.threshold(total_rgb_img, 600.0, 65535.0, cv2.THRESH_BINARY_INV)[1])
        # Set pixels with a total RGB value below 150 to black
        thresh_lower = np.uint8(cv2.threshold(total_rgb_img, 150.0, 65535.0, cv2.THRESH_BINARY)[1])
        # Set pixels to black if the difference between red and green is less than 80
        thresh_diff = np.uint8(cv2.threshold(np.abs(np.int16(G) - np.int16(R)), 80.0, 65535.0, cv2.THRESH_BINARY)[1])

        # apply mask
        mask = thresh_upper & thresh_lower & thresh_diff
        cropped_img &= cv2.merge((mask, mask, mask))

        # Comparison of the number of red and green pixels
        total_red = cv2.sumElems(cropped_img[:, :, 2])
        total_green = cv2.sumElems(cropped_img[:, :, 1])

        if total_red > total_green:
            self.drive = False
        else:
            self.drive = True

        # Publish message
        msg = Bool()
        msg.data = self.drive
        self.traffic_light_pub.publish(msg)

    def street_sign_processing(self, cropped_img):
        # Adjust and normalize image size
        input_img = cv2.resize(cropped_img, (96, 80))
        input_img = np.array(input_img).astype(np.float32)
        input_img = input_img / 255.0  # Normalize pixel values
        input_img = np.expand_dims(input_img, axis=0)  # add Batch-Dimension

        # prediction
        prediction = model.predict(input_img)

        cv2.imshow("img", cropped_img)
        cv2.waitKey(1)

        print(prediction)

        predicted_class = np.argmax(prediction, axis=1)
        print(predicted_class)

        #Pub sign
        class_msg = Int32()
        class_msg.data = int(predicted_class[0])
        self.sign_pub.publish(class_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
