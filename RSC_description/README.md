# RSC_description

## Features

- **Robot Visualization:** View the virtual twin of the RSC in **RViz2**.
- **Customizable Appearance:** Change the robot’s colors and facial expressions.

## Installation

### Prerequisites

Ensure you have the following installed:

- **ROS 2 Humble**
- **RViz2** for visualization

### Steps

1. Clone the repository into your ROS 2 workspace:
   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/RSC_description.git
   ```
2. Build the package:
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select RSC_description
   ```
3. Source the workspace:
   ```sh
   source install/setup.bash
   ```
4. Launch the visualization:
   ```sh
   ros2 launch RSC_description display_robot.launch.py
   ```
5. Launch the visualization with **Torso movement**:
   ```sh
   ros2 launch RSC_description display_torso.launch.py
   ```

## Usage


### Changing Colors and Faces

Modify the robot’s appearance (the face emotion and the buttun color) by modifying lines 4 and 5 values in **src/RSC_description/urdf/main.urdf.xacro** or **src/RSC_description/urdf/torso.urdf.xacro**

### Controlling Flippers

Run the following node to directly control the flippers:

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```



