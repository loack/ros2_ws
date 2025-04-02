import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CommandLineStatePublisher(Node):
    def __init__(self, namespace):
        super().__init__('command_line_state_publisher')
        self.namespace = namespace
        self.publisher = self.create_publisher(JointState, f'/{namespace}/joint_states', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_states)
        self.joint_names = [f'joint_{i+1}' for i in range(6)]
        self.joint_positions = [0.0] * 6  # Initialize all joint angles to 0

    def publish_joint_states(self):
        # Create and publish the JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint states: {self.joint_positions}')

    def update_joint_states(self):
        # Ask the user for joint angles
        try:
            input_angles = input(f"Enter 6 joint angles (space-separated) for namespace '{self.namespace}': ")
            angles = [float(angle) for angle in input_angles.split()]
            if len(angles) != 6:
                raise ValueError("You must provide exactly 6 angles.")
            self.joint_positions = angles
            self.get_logger().info(f'Updated joint states to: {self.joint_positions}')
        except ValueError as e:
            self.get_logger().error(f"Invalid input: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Ask the user for the namespace
    namespace = input("Enter the namespace (e.g., 'remus' or 'romulus'): ")

    node = CommandLineStatePublisher(namespace)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_joint_states()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()