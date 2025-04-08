import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

#!/usr/bin/env python3

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.get_logger().info(f'Initializing {self.get_name()} node')

        # Publisher for remus and romulus joint states
        self.remus_joint_state_publisher = self.create_publisher(JointState, '/remus/joint_states', 10)
        self.romulus_joint_state_publisher = self.create_publisher(JointState, '/romulus/joint_states', 10)
        self.get_logger().info(f'Publishing joint states on /remus/joint_states and /romulus/joint_states')
        
        # Initialize joint names and positions
        self.remus_joint_names = ['remus_joint_a1', 
                                  'remus_joint_a2',
                                  'remus_joint_a3',
                                  'remus_joint_a4',
                                  'remus_joint_a5',
                                  'remus_joint_a6']
        self.remus_joint_positions = [0.0] * len(self.remus_joint_names)  # Initialize all joints to 0.5

        # Initialize joint names and positions
        self.romulus_joint_names = ['romulus_joint_a1', 
                                  'romulus_joint_a2',
                                  'romulus_joint_a3',
                                  'romulus_joint_a4',
                                  'romulus_joint_a5',
                                  'romulus_joint_a6']
        self.romulus_joint_positions = [0.0] * len(self.romulus_joint_names)  # Initialize all joints to 0.5

        # Publish joint states at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        # Create and populate the JointState message for remus
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.remus_joint_names  # List of joint names (strings)
        joint_state_msg.position = self.remus_joint_positions  # List of joint positions (floats)

        # Publish the message
        self.remus_joint_state_publisher.publish(joint_state_msg) 
        self.get_logger().info(f'Published joint states: {self.remus_joint_positions}')

        # Create and populate the JointState message for romulus
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.romulus_joint_names
        joint_state_msg.position = self.romulus_joint_positions
        # Publish the message
        self.romulus_joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {self.romulus_joint_positions}')
        # Update joint positions for the next iteration
        # Example: Increment the joint positions for demonstration purposes
        for i in range(len(self.remus_joint_positions)):
            self.remus_joint_positions[i] += 0.1
            self.romulus_joint_positions[i] += 0.1
        # Reset joint positions if they exceed a certain limit
        if any(pos > 1.0 for pos in self.remus_joint_positions):
            self.remus_joint_positions = [0.0] * len(self.remus_joint_names)
        if any(pos > 1.0 for pos in self.romulus_joint_positions):
            self.romulus_joint_positions = [0.0] * len(self.romulus_joint_names)
        # Log the current joint positions
        self.get_logger().info(f'Remus joint positions: {self.remus_joint_positions}')
        self.get_logger().info(f'Romulus joint positions: {self.romulus_joint_positions}')
        # Sleep for a short duration to control the publishing rate
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()