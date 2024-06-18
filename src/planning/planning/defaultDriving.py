import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from interfaces.msg import Lanes
from interfaces.srv import FollowLane

from std_msgs.msg import String


class DefaultDriving(Node):

    def __init__(self):
        super().__init__('default_driving')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', -12)
        self.declare_parameter('boundary_right', 12)
        self.declare_parameter('speed_drive', 0.05)
        self.declare_parameter('speed_turn', 0.0055)
        
        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                         history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                         depth=1)
        
        # create subscriber for road lanes
        self.lane_subscription = self.create_subscription(
            Lanes,
            'lanes',
            self.lanes_callback,
            10)
        self.m = None
        self.x_sum = 0
        self.last_value = 0.0

        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        
        # create service to update lane following
        self.lane_service = self.create_service(FollowLane, 'follow_lanes', 
                                                self.follow_lanes_callback)

        self.get_logger().info('initialized defaultDriving')


    def lanes_callback(self, msg):
        if len(msg.right) >= 3:
            x1 = msg.right[0].x
            x2 = msg.right[1].x
            x3 = msg.right[2].x
            self.m = (x1 + x2 + x3) / 3
        else:
            self.m = None

        self.x_sum = 0
        num_x_values = len(msg.right)
        if num_x_values > 0:
            for lane_point in msg.right:
                self.x_sum += abs(lane_point.x)
            self.x_sum = self.x_sum / num_x_values
        else:
            self.x_sum = None


    def follow_lanes_callback(self, request, response):
        boundary_left   = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right  = self.get_parameter('boundary_right').get_parameter_value().integer_value
        speed_drive     = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn      = self.get_parameter('speed_turn').get_parameter_value().double_value

        turn = ((self.x_sum * speed_turn) + self.last_value) / 2
        self.last_value = turn

        speed_drive = (1 / (self.x_sum + 100)) * 150 * speed_drive

        #print("drive: %d turn: %d" % (speed_drive, speed_turn))
        if self.m is not None:
            if self.m < boundary_left:
                turn *= 1
                #self.get_logger().info('turn left (m = {})'.format(self.m))
            elif self.m > boundary_right:
                turn *= -1
                #self.get_logger().info('turn right (m = {})'.format(self.m))
            else: 
                turn *= 0
                #self.get_logger().info('drive straight (m = {})'.format(self.m))

            msg = Twist()
            msg.linear.x = speed_drive
            msg.angular.z = turn
            self.velocity_publisher.publish(msg)

        #else:
            #self.get_logger().info('Did not receive valid lane messages')

        return response



def main(args=None):
    rclpy.init(args=args)

    default_driving = DefaultDriving()

    rclpy.spin(default_driving)

    default_driving.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()