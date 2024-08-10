import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

class ArmCalibration(Node):
    def __init__(self):
        super().__init__('arm_calibration')
        
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.calibration_mode = False
        self.known_positions = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0,
            'joint_5': 0.0,
            'joint_6': 0.0,
            'gripper_joint': 0.0
        }
        self.offsets = {key: 0.0 for key in self.known_positions.keys()}
        self.relative_positions = {key: 0.0 for key in self.known_positions.keys()}
        
        self.current_joint = 'joint_1'
        self.increment = 0.4
        
        self.create_subscription(JointState, '/raw/joint_states', self.joint_state_callback, 10)
        
        self.get_logger().info("Press 'c' to toggle calibration mode. Use '1' to '6' to select joints. Use 'i' to increment and 'd' to decrement position. Press 's' to save offsets.")
        self.keyboard_input()

    def keyboard_input(self):
        tty.setcbreak(sys.stdin)
        while True:
            char = sys.stdin.read(1)
            if char == 'c':
                self.calibration_mode = not self.calibration_mode
                self.get_logger().info(f"Calibration mode {'enabled' if self.calibration_mode else 'disabled'}.")
            elif char in '1234567':
                if char == '7':
                    self.current_joint = 'gripper_joint'
                else:
                    self.current_joint = f'joint_{char}'
                
                # Reset the increment to default when changing joints
                self.increment = 0.4
                if self.current_joint == 'gripper_joint':
                    self.increment = 0.1
                    
                self.get_logger().info(f"Selected {self.current_joint}. Increment: {self.increment:.2f} rad")
            elif char == 'i' and self.calibration_mode:
                self.send_incremental_command(self.current_joint, self.increment)
            elif char == 'd' and self.calibration_mode:
                self.send_incremental_command(self.current_joint, -self.increment)
            elif char == 's' and self.calibration_mode:
                self.save_offsets()
            elif char == '+':
                self.increment += 0.1
                self.get_logger().info(f"Increment increased to {self.increment:.2f} rad")
            elif char == '-':
                self.increment = max(0.1, self.increment - 0.1)  # Prevent decrement below 0.1
                self.get_logger().info(f"Increment decreased to {self.increment:.2f} rad")
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
