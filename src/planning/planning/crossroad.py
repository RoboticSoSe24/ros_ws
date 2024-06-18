import rclpy
from rclpy.node import Node

from interfaces.srv import CrossIntersection

import time


class Crossroad(Node):

    def __init__(self):
        super().__init__('crossroad')

        self.service = self.create_service(
            CrossIntersection,
            'cross_intersection',
            self.cross_callback)
        
        self.get_logger().info('initialized Crossroad')


    def cross_callback(self, request, response):
        self.get_logger().info('cross intersection in direction:')
        match request.sign:
            case 0:
                self.get_logger().info('left')
            case 1:
                self.get_logger().info('straight')
            case 2:
                self.get_logger().info('right')
            case _:
                self.get_logger().info('unknown direction requested')
        return response



def main(args=None):
    rclpy.init(args=args)
    crossroad = Crossroad()
    rclpy.spin(crossroad)
    crossroad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
