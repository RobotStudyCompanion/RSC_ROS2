# RSC_movements_emotions

## Features

- **Digital Twin Control:** View the emotional movements of the digital twin of the RSC in **RViz2**.
- **Customizable Motions:** Change the robot’s motions or create your own.
- **Movements for Torso Digital Twin:** There are emotional movements for the second digital twin with the torso rotational joint.

## Usage

### Prerequisites

Ensure you have the following installed:

- **rsc_description** package
- **ROS 2 Humble**
- **RViz2** for visualization

### Steps

1. Clone the repositories into your ROS 2 workspace:

2. Build the packages:
   ```sh
   colcon build --packages-select rsc_description
   colcon build --packages-select rsc_movements_emotions
   ```
3. Source the workspace:
   ```sh
   source install/setup.bash
   ```
4. Launch the visualization:
   ```sh
   ros2 launch rsc_description display_robot.launch.py
   ```
5. Launch your emotional movement:
    ```sh
    ros2 run digital_twin joy.py  --  fun.py  --  caring.py  --  pride.py  --  anger.py  --  surprise.py  --  neutral.py
    ```
5. Launch your emotional movement for the **Torso Digital Twin**:
   ```sh
   ros2 run digital_twin joyT.py  --  funT.py  --  caringT.py  --  prideT.py  --  angerT.py  --  surpriseT.py  --  neutralT.py
   ```



