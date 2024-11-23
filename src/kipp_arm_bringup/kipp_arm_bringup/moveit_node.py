import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test')
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/commands', 10)
        self.timer = self.create_timer(0.1, self.publish_command)  # 10 Hz timer

        self.twist = TwistStamped()
        self.twist.twist.linear.x = 0.1  # Move along the x-axis
        self.twist.twist.angular.z = 0.1  # Rotate around the z-axis

    def publish_command(self):
        self.publisher.publish(self.twist)
        self.get_logger().info('Publishing servo command')

def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
