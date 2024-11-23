import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.get_logger().info("Use WASD keys to control the robot. Press Q to quit.")

        while True:
            key = input("Enter command (WASD): ").strip().lower()
            if key == 'q':
                break

            twist = TwistStamped()
            if key == 'w':
                twist.twist.linear.x = 0.1
            elif key == 's':
                twist.twist.linear.x = -0.1
            elif key == 'a':
                twist.twist.linear.y = 0.1
            elif key == 'd':
                twist.twist.linear.y = -0.1
            else:
                continue

            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
