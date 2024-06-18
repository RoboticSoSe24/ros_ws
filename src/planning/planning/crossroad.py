import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from interfaces.srv import CrossIntersection

import time


class Crossroad(Node):

    def __init__(self):
        super().__init__('crossroad')
        self.get_logger().info('initialized Crossroad')

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.service = self.create_service(
            CrossIntersection,
            'cross_intersection',
            self.cross_callback)


    def cross_callback(self, request, response):

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
        msg.angular.z = -0.23
        self.velocity_publisher.publish(msg)
        time.sleep(5)

    def left_corner(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(1)
        
        msg.linear.x = 0.1
        msg.angular.z = 0.145
        self.velocity_publisher.publish(msg)
        time.sleep(9)

    def no_corner(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)
        time.sleep(11)

    #def stop(self):
    #    msg = Twist()
    #    msg.linear.x = 0.0
    #    msg.angular.z = 0.0
    #    self.velocity_publisher.publish(msg)
    #    time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    crossroad = Crossroad()
    rclpy.spin(crossroad)
    crossroad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
