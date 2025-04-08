import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

#!/usr/bin/env python3

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.get_logger().info(f'Initializing {self.get_name()} node')

        # Publishers for remus and romulus joint states
        self.joint_state_publishers = {
            'remus': self.create_publisher(JointState, '/remus/joint_states', 10),
            'romulus': self.create_publisher(JointState, '/romulus/joint_states', 10)
        }
        self.get_logger().info(f'Publishing joint states on /remus/joint_states and /romulus/joint_states')
        
        # Initialize joint names and positions
        self.joint_data = {
            'remus': {
                'names': ['remus_joint_a1', 'remus_joint_a2', 'remus_joint_a3', 
                          'remus_joint_a4', 'remus_joint_a5', 'remus_joint_a6'],
                'positions': [0.0] * 6
            },
            'romulus': {
                'names': ['romulus_joint_a1', 'romulus_joint_a2', 'romulus_joint_a3', 
                          'romulus_joint_a4', 'romulus_joint_a5', 'romulus_joint_a6'],
                'positions': [0.0] * 6
            }
        }

        # Publish joint states at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.timer2 = self.create_timer(0.1, self.rotate_robots)

    def publish_joint_states(self):
        self.publish_robot_joint_states('remus')
        self.publish_robot_joint_states('romulus')

    def publish_robot_joint_states(self, robot_name):
        # Create and populate the JointState message for the given robot
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_data[robot_name]['names']
        joint_state_msg.position = self.joint_data[robot_name]['positions']

        # Publish the message
        self.joint_state_publishers[robot_name].publish(joint_state_msg)
        self.get_logger().info(f'Published {robot_name.capitalize()} joint states: {self.joint_data[robot_name]["positions"]}')
    
    def change_joint_state(self, robot_name, joint_index, new_position):
        # Change the joint state for the specified robot and joint index
        if robot_name in self.joint_data and 0 <= joint_index < len(self.joint_data[robot_name]['positions']):
            self.joint_data[robot_name]['positions'][joint_index] = new_position
            self.get_logger().info(f'Changed {robot_name.capitalize()} joint {joint_index} to {new_position}')
        else:
            self.get_logger().error(f'Invalid robot name or joint index: {robot_name}, {joint_index}')

    def rotate_robots(self):
        # Rotate the robots by changing their joint states
        self.joint_data['remus']['positions'][0] += 0.1
        self.joint_data['romulus']['positions'][0] += 0.1
        # Log the new joint states
        self.get_logger().info(f'Rotated Remus joint states: {self.joint_data["remus"]["positions"]}')
        self.get_logger().info(f'Rotated Romulus joint states: {self.joint_data["romulus"]["positions"]}')


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