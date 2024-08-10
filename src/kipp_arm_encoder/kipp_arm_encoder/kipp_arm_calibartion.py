import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64  # Import Float64 message type
import sys
import termios
import tty

class ArmCalibration(Node):
    def __init__(self):
        super().__init__('arm_calibration')
        
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_velocity_publisher = self.create_publisher(Float64, '/gripper_controller/velocity', 10)  # Publisher for gripper velocity
        
        self.calibration_mode = False
        self.known_positions = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0,
            'joint_5': 0.0,
            'joint_6': 0.0
        }
        self.offsets = {key: 0.0 for key in self.known_positions.keys()}
        self.relative_positions = {key: 0.0 for key in self.known_positions.keys()}
        
        self.current_joint = 'joint_1'
        self.increment = 0.4
        self.gripper_velocity = 0.2  # Default velocity value for the gripper
        
        self.create_subscription(JointState, '/raw/joint_states', self.joint_state_callback, 10)
        
        self.get_logger().info("Press 'c' to toggle calibration mode. Use '1' to '6' to select joints. Use 'i' to increment and 'd' to decrement position. Press 's' to save offsets.")
        self.get_logger().info("Press 'o' to open the gripper and 'p' to close the gripper. Release the key to stop.")
        
        self.keyboard_input()

    def keyboard_input(self):
        tty.setcbreak(sys.stdin)
        while True:
            char = sys.stdin.read(1)
            if char == 'c':
                self.calibration_mode = not self.calibration_mode
                self.get_logger().info(f"Calibration mode {'enabled' if self.calibration_mode else 'disabled'}.")
            elif char in '123456':
                self.current_joint = f'joint_{char}'
                self.get_logger().info(f"Selected {self.current_joint}.")
            elif char == 'i' and self.calibration_mode:
                self.send_incremental_command(self.current_joint, self.increment)
            elif char == 'd' and self.calibration_mode:
                self.send_incremental_command(self.current_joint, -self.increment)
            elif char == 's' and self.calibration_mode:
                self.save_offsets()
            elif char == 'o':  # Open gripper
                self.send_gripper_velocity(self.gripper_velocity)
            elif char == 'p':  # Close gripper
                self.send_gripper_velocity(-self.gripper_velocity)
            elif char in ['o', 'p']:  # Stop gripper on release
                self.send_gripper_velocity(0.0)
            elif char == 'q':
                break

    def send_incremental_command(self, joint, increment):
        # Calculate the new position
        new_position = self.known_positions[joint] + increment
        
        # Update the known position
        self.known_positions[joint] = new_position
        
        # Create a JointTrajectory message to move the arm
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [new_position]
        point.time_from_start.sec = 1  # Adjust as needed for smooth movement
        joint_trajectory.points = [point]
        
        # Publish the trajectory to move the joint
        self.joint_trajectory_publisher.publish(joint_trajectory)
        self.get_logger().info(f"Sent command to {joint}: {new_position:.2f} rad")

    def send_gripper_velocity(self, velocity):
        # Create a Float64 message to set gripper velocity
        velocity_msg = Float64()
        velocity_msg.data = velocity
        
        # Publish the velocity command
        self.gripper_velocity_publisher.publish(velocity_msg)
        self.get_logger().info(f"Sent gripper velocity command: {velocity:.2f}")

    def save_offsets(self):
        self.get_logger().info("Saving offsets...")
        for joint, known_position in self.known_positions.items():
            # Calculate the offset based on the difference between known and current positions
            offset = self.relative_positions[joint] - known_position
            self.offsets[joint] = offset
            self.get_logger().info(f"Offset for {joint}: {offset:.2f} rad")

        # Optionally, save offsets to a file or parameter server for future use
        # self.save_to_file(self.offsets)

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.relative_positions:
                self.relative_positions[name] = pos

def main(args=None):
    rclpy.init(args=args)
    arm_calibration_node = ArmCalibration()
    rclpy.spin(arm_calibration_node)
    
    arm_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
