import rclpy
from rclpy.node import Node
from interfaces.srv import FollowLane

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import time

class obstruction(Node):
    def __init__(self):
        super().__init__('obstruction')

        self.declare_parameter('scan_angle', 100.0)
        self.declare_parameter('buffer', 1.0)
        self.declare_parameter('object_distance', 0.5)

        # variable for the last sensor reading
        self.right_object = True
        self.did_turn = False
        self.wait_counter = 0

        self.m = None

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
        
        # create publisher for driving commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(timer_period, self.follow_and_switch)

        # create client to call default driving
        self.driving_client = self.create_client(FollowLane, 'follow_lanes')

        self.get_logger().info('initialized obstruction')



    def scanner_callback(self, msg):
        scan_angle = self.get_parameter('scan_angle').get_parameter_value().double_value
        object_distance = self.get_parameter('object_distance').get_parameter_value().double_value
        buffer = self.get_parameter('buffer').get_parameter_value().double_value

        sum_for_True = 0
        laser_data = msg.ranges  # array of all laser data

        for i in range(280 - int(scan_angle / 2), 280 + int(scan_angle / 2)):
            if laser_data[i] < object_distance and laser_data[i] != 0.0:
                sum_for_True += 1

        if sum_for_True > buffer:
            self.right_object = True
        else:
            self.right_object = False

        self.get_logger().info('laserscan: ' + str(self.right_object))

        
    def follow_and_switch(self):

        msg = Twist()
        if self.did_turn == False:
            self.get_logger().info('Left Turn')
            #drehe um 90 Grad nach links
            speed = 0.0
            msg.linear.x = speed
            msg.angular.z = 0.75 #0.82
            self.velocity_publisher.publish(msg)
            time.sleep(2)

            #fahre danach für 20 cm nach vorn
            speed = 0.15
            msg.linear.x = speed
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)
            time.sleep(2)

            #drehe um 90 Grad nach rechts
            speed = 0.0
            msg.linear.x = speed
            msg.angular.z = -0.75 #-0.82
            self.velocity_publisher.publish(msg)
            time.sleep(2)

            #fahre neben die Box
            speed = 0.15
            msg.linear.x = speed
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)
            time.sleep(0.2)

            speed = 0.0
            msg.linear.x = speed
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)

            self.did_turn = True
            return 

        if self.wait_counter < 10:
            self.get_logger().info('waiting: ' + str(self.wait_counter))
            self.wait_counter += 1
            return

        if self.right_object == True:
            self.get_logger().info('Enter default Driving')
            req = FollowLane.Request()
            req.right_lane = True
            future = self.driving_client.call_async(req)
            future.done() 

        else: 
            #msg = Twist()
            speed = 0.0
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)

            self.get_logger().info('Right Turn')

            #drehe um 90 Grad nach rechts
            speed = 0.0
            msg.linear.x = speed
            msg.angular.z = -0.82
            self.velocity_publisher.publish(msg)
            time.sleep(2)

            #fahre danach für 20 cm nach vorn
            speed = 0.15
            msg.linear.x = speed
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)
            time.sleep(2)

            #drehe um 90 Grad nach links
            speed = 0.0
            msg.linear.x = speed
            msg.angular.z = 0.82
            self.velocity_publisher.publish(msg)
            time.sleep(2)

            speed = 0.0
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)
        
            exit()



def main(args=None):
    print("obstruction test")
    rclpy.init(args=args)

    obstruction_Node = obstruction()

    rclpy.spin(obstruction_Node)

    obstruction_Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()