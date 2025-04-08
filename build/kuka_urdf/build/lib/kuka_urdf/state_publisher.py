#!./.venv/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time, os
import evdev
from evdev import InputDevice, categorize, ecodes, list_devices

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

        self.publish_joint_states()
        
        # Publish joint states at a fixed rate
        self.timer = self.create_timer(0.01, self.publish_joint_states)

        # Initialize the Xbox controller
        self.controller = self.find_controller()
        if self.controller:
            self.controller.grab()
        else:
            self.get_logger().error('No Xbox controller found. Joint states will not be updated.')
        
        self.joystick_state = {
            "ABS_X": 0,
            "ABS_Y": 0,
            "ABS_RX": 0,
            "ABS_RY": 0,
            "ABS_Z": 0,
            "ABS_RZ": 0,
            "BTN_SOUTH": 0,  # A
            "BTN_EAST": 0,   # B
            "BTN_NORTH": 0,  # X
            "BTN_WEST": 0,   # Y
            "BTN_TL": 0,     # LB
            "BTN_TR": 0,     # RB
            "BTN_THUMBL": 0,
            "BTN_THUMBR": 0,
            "BTN_START": 0,
            "BTN_SELECT": 0,
            "ABS_HAT0X": 0,
            "ABS_HAT0Y": 0,
        }

        #lauch joystick reading in a separate thread
        self.joystick_thread = self.create_timer(0.01, self.read_joystick)
        self.get_logger().info('Joystick reading thread started')
    
    def change_joint_state_values(self):
        # Change joint states based on joystick input
        if self.joystick_state["ABS_RX"] > 0:
            self.change_joint_state('remus', 0, self.joint_data['remus']['positions'][0] + 0.1)
        elif self.joystick_state["ABS_RX"] < 0:
            self.change_joint_state('remus', 0, self.joint_data['remus']['positions'][0] - 0.1)

        if self.joystick_state["ABS_RY"] > 0:
            self.change_joint_state('remus', 1, self.joint_data['remus']['positions'][1] + 0.1)
        elif self.joystick_state["ABS_RY"] < 0:
            self.change_joint_state('remus', 1, self.joint_data['remus']['positions'][1] - 0.1)

    def read_joystick(self):
        # Read joystick events
        for event in self.controller.read_loop():
            if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:
                name = ecodes.bytype[event.type][event.code]
                self.joystick_state[name] = event.value
                print(self.joystick_state)
                #update joint states based on joystick input
                self.change_joint_state_values()

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

    def find_controller(self):  # Add 'self' as the first parameter
        print("Recherche d'une manette Xbox connectée...")
        devices = [InputDevice(path) for path in list_devices()]
        for device in devices:
            if 'Xbox' in device.name or 'Controller' in device.name:
                print(f"✅ Manette trouvée : {device.name} ({device.path})")
                return device
        print("❌ Aucune manette trouvée.")
        return None


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
        # Cleanup
        node.controller.ungrab()
        node.controller.close()
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()