import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class JointPublisher(Node):
    def __init__(self):
        super().__init__('remus_joints_publisher')
        self.publisher_ = self.create_publisher(JointState, '/remus/joint_states', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish every 0.1 seconds
        self.remus_joint_a2_position = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['remus_joint_a1']
        msg.position = [self.remus_joint_a2_position]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')
        self.remus_joint_a2_position += 0.02  # Increment joint1 position slowly
        if self.remus_joint_a2_position > 2 * math.pi:
            self.remus_joint_a2_position -= 2 * math.pi  # Keep the value within [0, 2π]

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()