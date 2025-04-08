# kuka_minus_cortex

This ROS2 package, `kuka_minus_cortex`, provides a simulation environment for two robotic models: `kuka_minus` and `kuka_cortex`. The package allows users to visualize and control these robots using a gamepad in RViz.

## Installation

To install the package, clone the repository into your ROS2 workspace:

```bash
git clone <repository_url>
cd <your_ros2_workspace>
colcon build
source install/setup.bash
```

## Launching the Simulation

To launch the simulation and visualize the robots in RViz, use the following command:

```bash
ros2 launch kuka_minus_cortex display.launch.py
```

This will start RViz with both `kuka_minus` and `kuka_cortex` loaded.

## Gamepad Configuration

The gamepad configuration is specified in the `config/gamepad.yaml` file. Ensure your gamepad is connected before launching the simulation. The mappings for the buttons and axes are defined in this configuration file, allowing you to control the robots effectively.

## URDF Models

The URDF models for the robots are located in the `urdf` directory:

- `kuka_minus.urdf`: Defines the structure and properties of the `kuka_minus` robot.
- `kuka_cortex.urdf`: Defines the structure and properties of the `kuka_cortex` robot.

## Dependencies

Make sure to have the following dependencies installed:

- ROS2 (Foxy, Galactic, or later)
- RViz
- Any additional dependencies specified in `package.xml`

## Contributing

Contributions to improve the package are welcome. Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.