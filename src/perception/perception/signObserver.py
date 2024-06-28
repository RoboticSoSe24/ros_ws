import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32

import cv2
from cv_bridge import CvBridge

import tensorflow as tf
import numpy as np



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



# Laden des Modells
model = tf.keras.models.load_model('./models/model_7.keras')

class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        # declare windows
        cv2.namedWindow("src image")
        cv2.moveWindow("src image", 1920, 400)
        cv2.namedWindow("traffic light")
        cv2.moveWindow("traffic light", 1920 + 400, 400)

        # declare parameters
        self.min_sign_count = TrackbarParameter(self, 'min_sign_count', "src image", 8, 30)
        self.color_diff     = TrackbarParameter(self, 'color_diff', "traffic light", 100, 255)
        self.min_brightness = TrackbarParameter(self, 'min_brightness', "traffic light", 200, 765)
        self.max_brightness = TrackbarParameter(self, 'max_brightness', "traffic light", 600, 765)

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data over WiFi
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # create subscribtion to image data with changed qos
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

        debug_img = img_cv.copy()

        # check cropped image for traffic lights and signs
        self.__find_traffic_light(cropped_img.copy(), debug_img)
        self.__find_traffic_sign(img_cv, debug_img)

        cv2.imshow("src image", debug_img)
        cv2.waitKey(1)


    def __find_traffic_light(self, cropped_img, debug_img=None):
        height, width, _ = cropped_img.shape

        B, G, R = cv2.split(cropped_img)

        total_rgb_img = np.zeros((height, width), dtype='uint16')
        total_rgb_img += B
        total_rgb_img += G
        total_rgb_img += R

        # Set pixels with a total RGB value above 600 to black
        thresh_upper = np.uint8(cv2.threshold(total_rgb_img, int(self.max_brightness), 65535.0, cv2.THRESH_BINARY_INV)[1])
        # Set pixels with a total RGB value below 150 to black
        thresh_lower = np.uint8(cv2.threshold(total_rgb_img, int(self.min_brightness), 65535.0, cv2.THRESH_BINARY)[1])
        # Set pixels to black if the difference between red and green is less than 80
        thresh_diff = np.uint8(cv2.threshold(np.abs(np.int16(G) - np.int16(R)), int(self.color_diff), 65535.0, cv2.THRESH_BINARY)[1])

        # apply mask
        mask = thresh_upper & thresh_lower & thresh_diff
        cropped_img &= cv2.merge((mask, mask, mask))

        # Comparison of the number of red and green pixels
        total_red = cv2.sumElems(cropped_img[:, :, 2])
        total_green = cv2.sumElems(cropped_img[:, :, 1])

        # color filter output
        cropped_img = cv2.resize(cropped_img, (200, 200))
        cv2.putText(cropped_img, "red" if (total_red > total_green) else "green", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255))
        cv2.imshow("traffic light", cropped_img)
        cv2.waitKey(1)

        # Publish message
        msg = Bool()
        msg.data = False if (total_red > total_green) else True
        self.traffic_light_pub.publish(msg)

        if debug_img is not None:
            cv2.rectangle(debug_img, (250,85), (290,130), (0,255,0), 2)
            cv2.putText(debug_img, 'light', (250,85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0))


    def __find_traffic_sign(self, img_cv, debug_img=None):
        # cut out multiple images to pass to CNN
        input_images = np.zeros((3, 50, 42))
        for i in range(3):
            if debug_img is not None:
                cv2.rectangle(debug_img, (250+i*8,85), (290+i*8,130), (0,0,255), 1)
            crop = img_cv[85 : 130, 250+i*8 : 290+i*8]          # cut out at slight offset
            crop = cv2.resize(crop, (42, 50))                   # adjust size
            crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)       # convert to grayscale
            crop = np.float32(np.array(crop)) / 255             # normalize pixel values
            input_images[i,:,:] = crop

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
        msg.data = int(pred_class if self.sign_count > int(self.min_sign_count) else 2)
        self.sign_pub.publish(msg)

        if debug_img is not None:
            cv2.putText(debug_img, 'sign', (250,75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255))



def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
