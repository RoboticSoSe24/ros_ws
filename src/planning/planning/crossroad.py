import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist

from interfaces.srv import CrossIntersection
from interfaces.srv import FollowLane

import time


class Crossroad(Node):

    def __init__(self):
        super().__init__('crossroad')

        # callback group to allow synchronous service calls
        self.cb_group = ReentrantCallbackGroup()

        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # create client to call default driving
        self.driving_client = self.create_client(
            FollowLane, 
            'follow_lanes',
            callback_group=self.cb_group)
        
        # create service for crossing intersection
        self.service = self.create_service(
            CrossIntersection,
            'cross_intersection',
            self.cross_callback)
        
        self.get_logger().info('initialized Crossroad')


    def cross_callback(self, request, response):
        self.get_logger().info('orienting before crossing intersection')
        
        msg = Twist()
        msg.linear.x = -0.1
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(1)

        for i in range(200):
            req = FollowLane.Request()
            req.right_lane = True
            req.orientation_only = True
            self.__sync_call(self.driving_client, req)
            time.sleep(0.01)

        self.get_logger().info('cross intersection in direction:')
        match request.sign:
            case CrossIntersection.Request.SIGN_LEFT:
                self.get_logger().info('left')
                self.left_corner()
            case CrossIntersection.Request.SIGN_STRAIGHT:
                self.get_logger().info('straight')
                self.no_corner()
            case CrossIntersection.Request.SIGN_RIGHT:
                self.get_logger().info('right')
                self.right_corner()
            case _:
                self.get_logger().info('unknown direction requested')
        return response


    def right_corner(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(2.5)

        msg.linear.x = 0.1
        msg.angular.z = -0.30
        self.velocity_publisher.publish(msg)
        time.sleep(5)

    def left_corner(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(2.5)
        
        msg.linear.x = 0.1
        msg.angular.z = 0.18
        self.velocity_publisher.publish(msg)
        time.sleep(8.5)

    def no_corner(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(11)

    def __sync_call(self, client, request):
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.01)
        return future.result()



def main(args=None):
    rclpy.init(args=args)

    crossroad = Crossroad()

    executor = MultiThreadedExecutor()
    executor.add_node(crossroad)

    executor.spin()

    executor.shutdown()
    crossroad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
