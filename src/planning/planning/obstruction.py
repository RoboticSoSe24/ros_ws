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
        self.declare_parameter('scan_direction', 290.0)

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
        direction   = self.get_parameter('scan_direction').get_parameter_value().double_value
        fov         = self.get_parameter('scan_angle').get_parameter_value().double_value

        self.min_distance = float('inf')

        # find the closest object in a scan angle to the right of the bot
        for i in range(int(direction - fov/2), int(direction + fov/2)):
            d = msg.ranges[i]
            if d != 0.0 and d < self.min_distance:
                self.min_distance = d


    def obstruction_callback(self, request, response):        
        self.get_logger().info('switch left')
        self.__rotate_90_deg(True)
        self.__drive_straight(2)
        self.__rotate_90_deg()
        self.__stop()

        self.get_logger().info('pass obstruction')
        while self.min_distance < request.distance:
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