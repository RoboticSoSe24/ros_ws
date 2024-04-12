import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray

class DefaultDriving(Node):

    def __init__(self):
        super().__init__('default_driving')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', -10)
        self.declare_parameter('boundary_right', 10)
        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('speed_turn', 0.5)

        self.m = 0
        
        # Initialize m as None
        self.m = None

        self.subscription = self.create_subscription(
            PoseArray,
            'lanes',  # Name des Topics
            self.listener_callback,
            10)
        self.subscription

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        if len(msg.poses) >= 2:
            x1 = msg.poses[0].position.x
            x2 = msg.poses[1].position.x
            self.m = (x1 + x2) / 2
            #self.get_logger().info('x={}'.format(self.m))

    # driving logic
    def timer_callback(self):
        if self.m is not None:
            boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
            boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
            speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
            speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

            if self.m < boundary_left:
                self.get_logger().info('links fahren! m={}'.format(self.m))
                turn = speed_turn
            elif self.m > boundary_right:
                self.get_logger().info('rechts fahren! m={}'.format(self.m))
                turn = -speed_turn
            else: 
                self.get_logger().info('geradaus fahren! m={}'.format(self.m))
                turn = 0.0

            msg = Twist()
            msg.linear.x = speed_drive
            msg.angular.z = turn

            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Keine gültigen Pose-Nachrichten erhalten')


def main(args=None):
    print("Hello, !!! DRIVE MODE active now!!! ;)")
    rclpy.init(args=args)

    default_driving = DefaultDriving()

    rclpy.spin(default_driving)

    default_driving.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
