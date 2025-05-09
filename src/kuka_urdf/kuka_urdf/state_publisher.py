#!./.venv/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time, os
import evdev
from evdev import InputDevice, categorize, ecodes, list_devices
import threading

import ikpy
from ikpy.chain import Chain
from ikpy.link import URDFLink

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.get_logger().info(f'Initializing {self.get_name()} node')

        #retrieve urdf arguments passed in launch
        self.declare_parameter('urdf_1', '/content/kr210l150_remus.urdf')
        self.declare_parameter('urdf_2', '/content/kr210l150_romulus.urdf')
        self.urdf_1 = self.get_parameter('urdf_1').get_parameter_value().string_value
        self.urdf_2 = self.get_parameter('urdf_2').get_parameter_value().string_value
        self.get_logger().info(f'URDF 1: {self.urdf_1}')
        self.get_logger().info(f'URDF 2: {self.urdf_2}')


        #ikpy initialization
        self.remus_chain = Chain.from_urdf_file(self.urdf_1)
        self.romulus_chain = Chain.from_urdf_file(self.urdf_2)
        self.get_logger().info('IKPY initialized')
        
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

        # Initialize the Xbox controller
        self.controller = self.find_controller()
        if self.controller:
            self.controller.grab()
            # Start the joystick reading in a separate thread
            joystick_thread = threading.Thread(target=self.read_joystick, daemon=True)
            joystick_thread.start()
            self.get_logger().info('Joystick reading thread started')
        else:
            self.get_logger().error('No Xbox controller found. Joint states will not be updated.')
        
        self.joystick_state = {
            "ABS_X": 0, # -32768 to 32767
            "ABS_Y": 0, # -32768 to 32767
            "ABS_RX": 0,
            "ABS_RY": 0,
            "ABS_Z": 0,  #base 0- 1024
            "ABS_RZ": 0, #base 0- 1024
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
            "ABS_BRAKE": 0,
            "ABS_HAT0Y": 0,
        }

        # Publish joint states at a fixed rate
        self.timer = self.create_timer(0.01, self.publish_joint_states)

        #lauch joystick reading in a separate thread
        #self.joystick_thread = self.create_timer(0.01, self.read_joystick)
        #self.get_logger().info('Joystick reading thread started')
 
    
    def change_joint_state_values(self):
        trigger = 2000
        speed = 0.01
        # Change joint states based on joystick input
        if abs(self.joystick_state["ABS_X"])> trigger:
            self.joint_data["remus"]["positions"][0] += speed * self.joystick_state["ABS_X"] / 32768
        if abs(self.joystick_state["ABS_Y"])> trigger:
            self.joint_data["remus"]["positions"][1] += speed * self.joystick_state["ABS_Y"] / 32768

        self.joint_data["remus"]["positions"][2] += speed * self.joystick_state["ABS_Z"] / 1024 #base 1024
        self.joint_data["remus"]["positions"][2] -= speed * self.joystick_state["ABS_RZ"] / 1024 #base 1024

        if abs(self.joystick_state["ABS_RY"])> trigger:
            self.joint_data["remus"]["positions"][3] += speed * self.joystick_state["ABS_RX"] / 32768
        if abs(self.joystick_state["ABS_RY"])> trigger:
            self.joint_data["remus"]["positions"][4] += speed * self.joystick_state["ABS_RY"] / 32768

    #independ read joystick thread
    def read_joystick(self):
        # Read joystick events in a loop
        try:
            for event in self.controller.read_loop(): #if event change joystick state
                if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:
                    #self.get_logger().info(f'Event: {event}, updating joystick state')
                    name = ecodes.bytype[event.type][event.code]
                    self.joystick_state[name] = event.value
                    #self.get_logger().info(f'Joystick state updated: {self.joystick_state}')
                    # Update joint states based on joystick input
                    #print joystick state
                    #self.get_logger().info(f'Joystick state: {self.joystick_state}')
                    
        except Exception as e:
            self.get_logger().error(f'Error reading joystick: {e}')

    def publish_joint_states(self):
        #self.read_joystick()
        # Update joint states based on joystick input
        self.change_joint_state_values()
        # Publish joint states for both robots

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
        #self.get_logger().info(f'Published {robot_name.capitalize()} joint states: {self.joint_data[robot_name]["positions"]}')
    
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