import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
from rclpy.qos import QoSProfile
import evdev

class JoyPublisher(Node):

    def __init__(self):
        super().__init__('joy_publisher')

        # Initialize pygame for joystick input
        pygame.init()
        pygame.joystick.init()

        # Initialize publisher
        self.publisher_spear = self.create_publisher(Joy, '/SPEAR_Arm/Joy_Topic', QoSProfile(depth=10))
        self.threshold = 0.08  # Control threshold for stick
        self.timer = self.create_timer(0.05, self.publish_joystick_input)

        # Select joystick by serial number
        spear_serial = "SPEAR_ARM_SERIAL_NUMBER"  # Replace with your joystick's serial number
        self.joystick_spear_id = self.get_joystick_id_by_serial(spear_serial)

        if self.joystick_spear_id is None:
            self.get_logger().error('Could not find specified joystick.')
            rclpy.shutdown()

    def get_joystick_id_by_serial(self, serial):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if 'ID_SERIAL' in device.info and device.info['ID_SERIAL'] == serial:
                return self.get_joystick_index_by_name(device.name)
        return None

    def get_joystick_index_by_name(self, name):
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            if joystick.get_name() == name:
                return i
        return None

    def get_joystick_input(self, joystick_id):
        if joystick_id is None:
            return None

        # Get the specific joystick
        joystick = pygame.joystick.Joystick(joystick_id)
        joystick.init()

        # Get axes and buttons from the joystick
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        hats = joystick.get_hat(0)
        hats_float = tuple(float(x) for x in hats)
        axes.extend(hats_float)

        axes = [0.00 if abs(x) < self.threshold else x for x in axes]

        # Create Joy message
        joystick_input = Joy()
        joystick_input.header.stamp = self.get_clock().now().to_msg()
        joystick_input.axes = axes
        joystick_input.buttons = buttons

        return joystick_input

    def publish_joystick_input(self):
        # Get joystick input for the arm controller
        joystick_input_spear = self.get_joystick_input(self.joystick_spear_id)

        if joystick_input_spear is not None:
            self.publisher_spear.publish(joystick_input_spear)

def main(args=None):
    rclpy.init(args=args)

    joy_publisher = JoyPublisher()

    rclpy.spin(joy_publisher)

    joy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
