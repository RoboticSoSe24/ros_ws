import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from messages.msg import Lanes

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
        self.declare_parameter('speed_turn', 0.0055)
        self.declare_parameter('scan_angle', 60)

        # variable for the last sensor reading
        self.min_index = 0
        self.min_value = 0.0
        self.last_value = 0.0
        
        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                         history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                         depth=1)
        
        # create subscriber for laserscan ranges
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.laser_subscription  

        # create subscriber for road lanes
        self.lane_subscription = self.create_subscription(
            Lanes,
            'lanes',
            self.lanes_callback,
            10)
        self.lane_subscription
        self.m = None

        # create subscriber for road lanes
        self.speed_lane_subscription = self.create_subscription(
            Lanes,
            'lanes',
            self.speed_adjusment_callback,
            10)
        self.speed_lane_subscription
        self.x_sum = 0


        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        
        timer_period = 0.2  # secondsspeed_adjusment_callback
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

        # create publisher for current action state
        self.state_publisher = self.create_publisher(String, 'state', 1)
        self.driving = True

        self.get_logger().info('initialized defaultDriving')


    def scanner_callback(self, msg):
        scan_angle = self.get_parameter('scan_angle').get_parameter_value().integer_value
        
        laser_data = msg.ranges

        self.min_value = float('inf')
        self.min_index = 0

        for i in range(-int(scan_angle/2), int(scan_angle/2)):
            d = laser_data[i]
            if d != 0.0 and d < self.min_value:
                self.min_value = d
                self.min_index = i
            

    def lanes_callback(self, msg):
        if len(msg.right) >= 3:
            x1 = msg.right[0].x
            x2 = msg.right[1].x
            x3 = msg.right[2].x
            self.m = (x1 + x2 + x3) / 3
            self.get_logger().info('x={}'.format(self.m))

    def speed_adjusment_callback(self, msg):
            self.x_sum = 0
            num_x_values = len(msg.right)
            if num_x_values > 0:
                for lane_point in msg.right:
                    self.x_sum += abs(lane_point.x)
                self.x_sum = self.x_sum / num_x_values
                self.get_logger().info('x={}'.format(self.x_sum))
            else:
                self.x_sum = None


    # driving logic
    def timer_callback(self):
        distance_stop   = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        boundary_left   = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right  = self.get_parameter('boundary_right').get_parameter_value().integer_value
        #speed_drive     = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn      = self.get_parameter('speed_turn').get_parameter_value().double_value

        turn = ((self.x_sum * speed_turn) + self.last_value) / 2
        self.last_value = turn

        speed_drive = (1 / (self.x_sum + 100)) * 15

        if self.min_value < distance_stop:
            speed_drive = 0.0
            turn = 0.0

        if not self.driving and speed_drive > 0.0:
            self.driving = True
            msg = String()
            msg.data = 'driving'
            self.state_publisher.publish(msg)

        if self.driving and speed_drive == 0.0:
            self.driving = False
            msg = String()
            msg.data = 'stopping'
            self.state_publisher.publish(msg)

        #self.get_logger().info('x_sum= {})'.format(self.x_sum))
        #self.get_logger().info('Drive Speed: {} '.format(speed_drive))


        if self.m is not None:
            if self.m < boundary_left:
                self.get_logger().info('turn left (m = {})'.format(self.m))
            elif self.m > boundary_right:
                turn *= -1
                self.get_logger().info('turn right (m = {})'.format(self.m))
            else: 
                turn *= 0
                self.get_logger().info('drive straight (m = {})'.format(self.m))
        

            
            msg = Twist()
            msg.linear.x = speed_drive
            msg.angular.z = turn
        
            self.velocity_publisher.publish(msg)
        else:
            self.get_logger().info('Did not receive valid lane messages')


def main(args=None):
    rclpy.init(args=args)

    default_driving = DefaultDriving()

    rclpy.spin(default_driving)

    default_driving.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()