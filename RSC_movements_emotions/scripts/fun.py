#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Fun(Node):
    def __init__(self):
        super().__init__('fun')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body']
        self.joint_state.position = [0.0, 0.0] 

        # Define the max and min positions for the joints
        self.max_positions = [3.0, 3.0] 

        # Define the increment for smooth movement
        self.increment = -0.5

        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 200    # Number of shakes to perform
        self.get_logger().info('Motion started.')

    def control_motion(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_count<self.max_shakes:
            self.move_arms()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        self.publisher_.publish(self.joint_state)

    def move_arms(self):
        self.joint_state.position[0] += self.increment
        self.joint_state.position[1] -= self.increment
        self.shake_count+=1


def main(args=None):
    rclpy.init(args=args)
    joy_node = Fun()
    
    try:
        rclpy.spin(joy_node)
    except KeyboardInterrupt:
        pass

    joy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

