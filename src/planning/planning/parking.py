import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces.srv import FollowLane

from interfaces.srv import ParkingSpace

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

import time

class DriveOnParkingSlot(Node):
    def __init__(self):
        super().__init__('parking')

        # callback group to allow synchronous service calls
        self.cb_group = ReentrantCallbackGroup()

        # declaration of parameters that can be changed at runtime
        self.declare_parameter('scan_height', 0.34)
        self.declare_parameter('scan_width', 0.35)
        self.declare_parameter('parking_time', 10)
        
        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                         history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                         depth=1)
        
        # create subscriber for laserscan ranges
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy,
            callback_group=self.cb_group)

        # variable for the last laserscan reading
        self.min_distance = float('inf')

        # variable for number of scanners recognizing the obstacle
        self.counter = 0
        
        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # create client to call default driving
        self.driving_client = self.create_client(
            FollowLane, 
            'follow_lanes',
            callback_group=self.cb_group)

        # create service for parking
        self.service = self.create_service(
            ParkingSpace,
            'park_in_space',
            self.parking_callback)

        timer_period = 1  # seconds
        #self.timer = self.create_timer(timer_period, self.parking_callback)

        self.get_logger().info('initialized Parking')


    def scanner_callback(self, msg):
        self.counter = 0
        height = self.get_parameter('scan_height').get_parameter_value().double_value
        width = self.get_parameter('scan_width').get_parameter_value().double_value

        def length(a):

            x = min(height / 2     * abs(math.tan(a)), width)
            y = min(x         / abs(math.tan(a)), height / 2)

            return math.sqrt(x**2 + y**2)
        
        # find the closest object within the right box
        for a in range(181, 360):
            a_rad = math.radians(a)
            l = length(a_rad)
            d = msg.ranges[a]
            if d != 0.0 and d < l:
                self.counter += 1

        self.get_logger().info('Counter: ' + str(self.counter))


    def parking_callback(self, request, response):
        distance = self.get_parameter('parking_slot_distance').get_parameter_value().double_value
        parking_time = self.get_parameter('parking_time').get_parameter_value().integer_value

        self.get_logger().info('search for free parking slot')
        for i in range(0, 80):
            req = FollowLane.Request()
            req.right_lane = True
            self.__sync_call(self.driving_client, req)
            time.sleep(0.1)

        while self.counter != 0:
            req = FollowLane.Request()
            req.right_lane = True
            self.__sync_call(self.driving_client, req)

        self.get_logger().info('found free parking slot')
        self.get_logger().info('drive onto parking slot')    
        self.__rotate_90_deg()
        self.__drive_straight(2)
        self.__stop()

        self.get_logger().info('parking...')
        self.__stay_on_slot(parking_time)

        self.get_logger().info('leave parking slot')
        self.__rotate_90_deg()
        self.__rotate_90_deg()
        self.__drive_straight(2.2)
        self.__stop()
        self.__rotate_90_deg()
        self.__stop()

        return response


    def __sync_call(self, client, request):
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.01)
        return future.result()
    

    def __rotate_90_deg(self, ccw=False):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.82 if ccw else -0.82
        self.velocity_publisher.publish(msg)
        time.sleep(2)


    def __drive_straight(self, seconds):
        msg = Twist()
        msg.linear.x = 0.13
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(seconds)

    def __stay_on_slot(self, seconds):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(seconds)


    def __stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    parking = DriveOnParkingSlot()
    
    executor = MultiThreadedExecutor()
    executor.add_node(parking)

    executor.spin()

    executor.shutdown()
    parking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()