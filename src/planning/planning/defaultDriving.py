import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

class DefaultDriving(Node):

    def __init__(self):
        super().__init__('default_driving')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('boundary_left', -12)
        self.declare_parameter('boundary_right', 12)
        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('speed_turn', 0.35)
        self.declare_parameter('scan_angle', 30)

        # variable for the last sensor reading
        self.min_index = 0
        self.min_value = 0.0

        self.m = 0
        
        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                         history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                         depth=1)
        
        # create subscribers
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.laser_subscription  

        self.lane_subscription = self.create_subscription(
            PoseArray,
            'lanes',  # Name des Topics
            self.listener_callback,
            10)
        self.lane_subscription

        self.m = None
        
        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        
        timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    def scanner_callback(self, msg):
        laser_data = msg.ranges
        self.min_value = float('inf') 
        self.min_index = None
        #finding the smallest value except from 0.0, but still counting the index
        for i, x in enumerate(laser_data):
            if x != 0.0 and x < self.min_value:
                self.min_value = x
                self.min_index = i
        self.get_logger().info('laserscan callback')


    def listener_callback(self, msg):
        if len(msg.poses) >= 2:
            x1 = msg.poses[0].position.x
            x2 = msg.poses[1].position.x
            self.m = (x1 + x2) / 2
            #self.get_logger().info('x={}'.format(self.m))

    # driving logic
    def timer_callback(self):
        if self.m is not None:
            distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
            boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
            boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
            speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
            speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
            scan_angle = self.get_parameter('scan_angle').get_parameter_value().double_value

            if self.min_value < distance_stop and (self.min_index <= (scan_angle / 2) or self.min_index >= (360 - (scan_angle / 2))):
                speed_drive = 0.0
                speed_turn = 0.0
                self.get_logger().info(str(self.min_value))

            if self.m < boundary_left:
                self.get_logger().info('Objekt={} // Index={} // links fahren!    m={}'.format(round(float(self.min_value), 2), self.min_index, self.m))
                turn = speed_turn
            elif self.m > boundary_right:
                self.get_logger().info('Objekt={} // Index={} // rechts fahren!   m={}'.format(round(float(self.min_value), 2), self.min_index, self.m))
                turn = -speed_turn
            else: 
                self.get_logger().info('Objekt={} // Index={} // geradaus fahren! m={}'.format(round(float(self.min_value), 2), self.min_index, self.m))
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