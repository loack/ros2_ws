import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/remus/joint_states',
            self.joint_state_callback,
            10
        )
        self.previous_joint_states = None

    def joint_state_callback(self, msg):
        # Check if the joint states have changed
        if self.previous_joint_states != msg.position:
            self.previous_joint_states = msg.position
            self.get_logger().info(f'Joint states updated: {msg.position}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("Inside joint_state_listener.py")
    main()