import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

class DefaultDriving(Node):

    def __init__(self):
        super().__init__('default_driving')
        self.subscription = self.create_subscription(
            PoseArray,
            'lanes',  # Name des Topics
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        # Überprüfe, ob mindestens zwei Pose-Nachrichten vorhanden sind
        if len(msg.poses) >= 2:
            # Extrahiere die Koordinaten des ersten Punktes aus der ersten Pose-Nachricht
            x1 = msg.poses[0].position.x
            y1 = msg.poses[0].position.y
            z1 = msg.poses[0].position.z

            # Extrahiere die Koordinaten des zweiten Punktes aus der zweiten Pose-Nachricht
            x2 = msg.poses[1].position.x
            y2 = msg.poses[1].position.y
            z2 = msg.poses[1].position.z

            self.get_logger().info('First point: x={}, y={}, z={}'.format(x1, y1, z1))
            self.get_logger().info('Second point: x={}, y={}, z={}'.format(x2, y2, z2))

def main(args=None):
    rclpy.init(args=args)

    default_driving = DefaultDriving()

    rclpy.spin(default_driving)

    default_driving.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
