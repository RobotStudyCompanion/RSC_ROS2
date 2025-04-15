# Digital Twin

## Features

- **Robot Visualization:** View the virtual twin of the RSC in **RViz2**.
- **Customizable Appearance:** Change the robot’s colors and facial expressions.
- **Flipper Control:** Utilize ROS 2 nodes to move the robot’s flippers.
- **Modular Design:** Easily extend and integrate with other ROS 2-based projects.

## Installation

### Prerequisites

Ensure you have the following installed:

- **ROS 2 Humble**
- **RViz2** for visualization

### Steps

1. Clone the repository into your ROS 2 workspace:
   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/digital_twin.git
   ```
2. Build the package:
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select digital_twin
   ```
3. Source the workspace:
   ```sh
   source install/setup.bash
   ```
4. Launch the simulation:
   ```sh
   ros2 launch digital_twin display_robot.launch.py
   ```

## Usage

### Running the Simulation

To start the robot simulation in RViz2:

```sh
ros2 launch digital_twin display_robot.launch.py
```

### Changing Colors and Faces

Modify the robot’s appearance (the face emotion and the buttun color) by modifying lines 4 and 5 values in **src/digital_twin/urdf/my_robot.urdf.xacro**.

### Controlling Flippers

Run the following node to control the flippers with the different pre-set emotional movements:

```sh
ros2 run digital_twin emotion.py
```

or the single emotions:

```sh
ros2 run digital_twin joy.py  --  fun.py  --  caring.py  --  pride.py  --  anger.py  --  surprise.py  --  neutral.py
```

Run the following node to directly control the flippers:

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
## DIGITAL TWIN TORSO

There is a second package called **digital_twin_torso**, it has the same functionalities of digital_twin but it has another movement: the rotation of the torso.

## Contact

For questions or support, reach out via [**c**](mailto\:your.email@example.com)[**alafa@ut.ee**](mailto\:alafa@ut.ee) or open an issue in the repository.

---

Happy coding! 🚀


