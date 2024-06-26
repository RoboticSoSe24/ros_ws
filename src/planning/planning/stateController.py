import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces.srv import FollowLane
from interfaces.srv import OvertakeObstruction
from interfaces.srv import ParkingSpace
from interfaces.srv import CrossIntersection

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import LaserScan

import time
import math


class StateController(Node):

    def __init__(self):
        super().__init__('state_controller')

        # callback group to allow synchronous service calls
        self.cb_group = ReentrantCallbackGroup()

        # declaration of parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('scan_angle', 50)

        self.__create_subscriptions()   # topic subscriptions
        self.__create_publishers()      # topic publishers
        self.__create_clients()         # service clients

        # create timer for logic updates
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('initialized StateController')


    def __create_subscriptions(self):
        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)

        # subscribe to laserscan ranges
        self.laser_subscription = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scanner_callback, 
            qos_profile=qos_policy, 
            callback_group=self.cb_group)
        self.min_distance = float('inf')    # variable for the last laserscan reading
        self.scan_counter = 0

        # subscribe to traffic lights
        self.traffic_light_sub = self.create_subscription(
            Bool, 
            'traffic_light', 
            self.traffic_light_callback, 
            1, 
            callback_group=self.cb_group)
        self.traffic_light_clear = True     # variable for the last traffic light message

        # subscribe to traffic signs
        self.signs_subscription = self.create_subscription(
            Int32, 
            'signs', 
            self.signs_callback, 
            1, 
            callback_group=self.cb_group)
        self.last_sign = None               # variable for the last received sign message


    def __create_publishers(self):
        # create publisher for velocity
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # create publisher for current action state
        self.state_publisher = self.create_publisher(String, 'state', 1)
        self.last_state = ""


    def __create_clients(self):
        # create client to call default driving
        self.driving_client = self.create_client(
            FollowLane, 
            'follow_lanes', 
            callback_group=self.cb_group)

        # create client to call obstruction overtake
        self.obstruction_client = self.create_client(
            OvertakeObstruction, 
            'overtake_obstruction', 
            callback_group=self.cb_group)

        # create client to call parking
        self.parking_client = self.create_client(
            ParkingSpace, 
            'park_in_space', 
            callback_group=self.cb_group)

        # create client to cross intersection
        self.crossroad_client = self.create_client(
            CrossIntersection,
            'cross_intersection',
            callback_group=self.cb_group)


    def scanner_callback(self, msg):
        scan_angle = self.get_parameter('scan_angle').get_parameter_value().integer_value
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value

        self.scan_counter = 0

        # find the closest object in a scan angle in front of the bot
        for i in range(-int(scan_angle / 2), int(scan_angle / 2)):
            d = msg.ranges[i]
            if d != 0.0 and d < distance_stop:
                self.scan_counter += 1


    def traffic_light_callback(self, msg):
        self.traffic_light_clear = msg.data


    def signs_callback(self, msg):
        self.last_sign = msg.data


    def timer_callback(self):
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value

        # if there is a red light prevent all other actions
        if not self.traffic_light_clear:
            self.__pub_state('stopping in front of red light')
            self.__stop()

        # overtake obstruction
        elif self.scan_counter > 1 and self.obstruction_client.service_is_ready():
            self.__pub_state('overtaking obstruction')
            req = OvertakeObstruction.Request()
            self.get_logger().info('overtaking completed')
            req.distance = distance_stop * math.sqrt(2) + 0.08  # safety margin so overtake won't immediately return
            self.__sync_call(self.obstruction_client, req)

        # stop in front of obstruction
        elif self.min_distance < distance_stop and not self.obstruction_client.service_is_ready():
            self.__pub_state('stopping in front of obstruction')
            self.__stop()
            while self.min_distance < distance_stop and not self.obstruction_client.service_is_ready():
                time.sleep(0.01)

        # parking sign
        elif self.last_sign == 3 and self.parking_client.service_is_ready():  
            self.__pub_state('parking')
            req = ParkingSpace.Request()
            self.__sync_call(self.parking_client, req)
            self.get_logger().info('Parking completed, resetting last_sign')
            self.last_sign = None  # Reset the sign after parking

        # crossroad
        elif self.last_sign in [0, 1, 4] and self.crossroad_client.service_is_ready():
            self.__pub_state('crossing intersection')
            req = CrossIntersection.Request()
            match self.last_sign:
                case 0: req.sign = CrossIntersection.Request.SIGN_LEFT
                case 1: req.sign = CrossIntersection.Request.SIGN_STRAIGHT
                case 4: req.sign = CrossIntersection.Request.SIGN_RIGHT
            self.__sync_call(self.crossroad_client, req)

        # default driving
        elif self.driving_client.service_is_ready():
            self.__pub_state('default driving')
            req = FollowLane.Request()
            req.right_lane = True
            self.__sync_call(self.driving_client, req)

        # for lack of better ideas: stop
        else:
            self.__pub_state('default stopping')
            self.__stop()


    def __pub_state(self, text, log=True):
        if text is not self.last_state:
            msg = String()
            msg.data = text
            self.state_publisher.publish(msg)
            if log:
                self.get_logger().info(text)
        self.last_state = text


    def __sync_call(self, client, request):
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.01)
        return future.result()


    def __stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    state_controller = StateController()

    executor = MultiThreadedExecutor()
    executor.add_node(state_controller)

    executor.spin()

    executor.shutdown()
    state_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
