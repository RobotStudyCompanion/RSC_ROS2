# Digital Twin

## Features

- **Flipper Control:** Utilize ROS 2 nodes to move the robot’s flippers and torso.


## Installation

### Prerequisites

Ensure you have the following installed:

- **ROS 2 Humble**
- **RViz2** for visualization

### Steps

1. Clone the repository into your ROS 2 workspace:
   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/RSC_movements_emotions.git
   ```
2. Build the package:
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select RSC_movements_emotions
   ```
3. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## Controlling Movements



Run the following node to control the flippers with the different pre-set emotional movements:

```sh
ros2 run RSC_movements_emotions emotion.py
```

or the single emotions:

```sh
ros2 run RSC_movements_emotions joy.py  --  fun.py  --  caring.py  --  pride.py  --  anger.py  --  surprise.py  --  neutral.py
```

Run the following node to directly control the flippers:

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Torso movements

Run the following node to control the flippers and torso with the different pre-set emotional movements:

```sh
ros2 run RSC_movements_emotions emotionT.py
```

or the single emotions:

```sh
ros2 run RSC_movements_emotions joyT.py  --  funT.py  --  caringT.py  --  prideT.py  --  angerT.py  --  surpriseT.py  --  neutralT.py
```

Run the following node to directly control the flippers:

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Contact

For questions or support, reach out via [**c**](mailto\:your.email@example.com)[**alafa@ut.ee**](mailto\:alafa@ut.ee) or open an issue in the repository.



