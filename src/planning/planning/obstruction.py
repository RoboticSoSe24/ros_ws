import rclpy
from rclpy.node import Node

from interfaces.msg import Lanes

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class obstruction(Node):
    def __init__(self):
        super().__init__('obstruction')

        # variable for the last sensor reading
        self.min_index = 0
        self.min_value = 0.0
        
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
        self.m = None
        
        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(timer_period, self.follow_and_switch)

        # create publisher for current action state
        self.state_publisher = self.create_publisher(String, 'state', 1)
        self.driving = True

        self.get_logger().info('initialized obstruction')



    def scanner_callback(self, msg):
        laser_data = msg.ranges #array of all laser data

        #ckeck here if object is still on the right lane



    def lanes_callback(self, msg):

        #average of first three points to determinate on if to turn or not
        if len(msg.right) >= 3:
            x1 = msg.right[0].x
            x2 = msg.right[1].x
            x3 = msg.right[2].x
            self.m = (x1 + x2 + x3) / 3
            self.get_logger().info('x={}'.format(self.m))
        else:
            self.m = None

        #abs average summ of all values -> drive slow or fast
        self.x_sum = 0
        num_x_values = len(msg.right)
        if num_x_values > 0:
            for lane_point in msg.right:
                self.x_sum += abs(lane_point.x)
            self.x_sum = self.x_sum / num_x_values
        else:
            self.x_sum = None

        self.get_logger().info('initialized lanes_callback')



    def follow_and_switch(self):
        speed = 0.0
        turn = 0.0

        #driving logic here to switch lanes 
        #and stay on lane once switched (see follow_lanes_callback in DeflaultDriving)
        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn
        self.velocity_publisher.publish(msg)


def main(args=None):
    print("obstruction test")
    rclpy.init(args=args)

    obstruction_Node = obstruction()

    rclpy.spin(obstruction_Node)

    obstruction_Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()