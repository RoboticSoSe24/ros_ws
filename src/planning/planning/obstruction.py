import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces.srv import OvertakeObstruction
from interfaces.srv import FollowLane

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import time

class Obstruction(Node):
    def __init__(self):
        super().__init__('obstruction')

        # callback group to allow synchronous service calls
        self.cb_group = ReentrantCallbackGroup()

        # declaration of parameters that can be changed at runtime
        self.declare_parameter('scan_angle', 100.0)
        self.declare_parameter('buffer', 1.0)
        self.declare_parameter('object_distance', 0.5)

        # variable for the last sensor reading
        self.right_object = True
        self.did_turn = False
        self.wait_counter = 0

        self.m = None

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
        
        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # create client to call default driving
        self.driving_client = self.create_client(
            FollowLane, 
            'follow_lanes',
            callback_group=self.cb_group)

        # create service for obstruction overtake
        self.service = self.create_service(
            OvertakeObstruction,
            'overtake_obstruction',
            self.obstruction_callback)

        self.get_logger().info('initialized Obstruction')


    def scanner_callback(self, msg):
        scan_angle      = self.get_parameter('scan_angle').get_parameter_value().double_value
        object_distance = self.get_parameter('object_distance').get_parameter_value().double_value
        buffer          = self.get_parameter('buffer').get_parameter_value().double_value

        sum_for_True = 0
        laser_data = msg.ranges  # array of all laser data

        for i in range(280 - int(scan_angle / 2), 280 + int(scan_angle / 2)):
            if laser_data[i] < object_distance and laser_data[i] != 0.0:
                sum_for_True += 1

        if sum_for_True > buffer:
            self.right_object = True
        else:
            self.right_object = False


    def obstruction_callback(self, request, response):        
        self.get_logger().info('switch left')
        self.__rotate_90_deg(True)
        self.__drive_straight(2)
        self.__rotate_90_deg()
        self.__stop()

        self.get_logger().info('pass obstruction')
        while self.right_object:
            req = FollowLane.Request()
            req.right_lane = False
            self.__sync_call(self.driving_client, req)

        self.get_logger().info('switch right')
        self.__rotate_90_deg()
        self.__drive_straight(2)
        self.__rotate_90_deg(True)
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


    def __stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    obstruction = Obstruction()

    executor = MultiThreadedExecutor()
    executor.add_node(obstruction)

    executor.spin()

    executor.shutdown()
    obstruction.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()