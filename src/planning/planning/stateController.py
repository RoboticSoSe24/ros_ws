import rclpy
from rclpy.node import Node

from interfaces.srv import FollowLane
from interfaces.srv import OvertakeObstruction

from geometry_msgs.msg import Twist

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import time
from os import system


class StateController(Node):

    def __init__(self):
        super().__init__('state_controller')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.35)
        self.declare_parameter('scan_angle', 80)

        # variable for the last sensor reading
        self.min_index = 0
        self.min_value = float('inf')
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

        # create publisher for velocity
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # create publisher for current action state
        self.state_publisher = self.create_publisher(String, 'state', 1)

        # create client to call default driving
        self.driving_client = self.create_client(FollowLane, 'follow_lanes')

        # create client to call obstruction overtake
        self.obstruction_client = self.create_client(OvertakeObstruction, 'overtake_obstruction')

        # future object to wait for state completion
        self.future = None

        self.get_logger().info('initialized StateController')


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


    # check necessity for state updates
    def timer_callback(self):
        # block updates while waiting for a service to complete
        if not(self.future is None) and (not self.future.done()):  
            return

        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        state_msg = String()

        # check whether to enter a different behavioral state

        if self.min_value < distance_stop:              # overtake obstruction
            self.get_logger().info('calling obstruction')
            req = OvertakeObstruction.Request()
            req.distance = distance_stop
            #self.future = self.obstruction_client.call_async(req)
            system('ros2 run planning obstruction')
            self.get_logger().info('called obstruction')
            self.min_value = float('inf')
            state_msg.data = 'overtake obstruction'

        else:                                           # default driving
            req = FollowLane.Request()
            req.right_lane = True
            self.future = self.driving_client.call_async(req)
            #self.driving_client.call_async(req)
            state_msg.data = 'default driving'

        self.state_publisher.publish(state_msg)



def main(args=None):
    rclpy.init(args=args)

    state_controller = StateController()
    
    rclpy.spin(state_controller)

    state_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()