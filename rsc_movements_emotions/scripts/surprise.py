#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Surprise(Node):
    def __init__(self):
        super().__init__('surprise')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body']
        self.joint_state.position = [0.0, 0.0] 

        # Define the max and min positions for the joints
        self.max_positions = [3.0, 3.0] 
        self.min_positions = [2.2, 2.2] 

        # Define the increment for smooth movement
        self.increment = -0.2

        self.moving_up = True
        self.shake_up = False
        self.shake_count = 0   
        self.max_shakes = 23 
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.moving_up:
            self.move_arms_to_top()
        elif self.shake_up and self.shake_count<self.max_shakes:
            self.shake_arms_top()
        elif self.shake_count<self.max_shakes:
            self.shake_arms_down()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        self.publisher_.publish(self.joint_state)

    def move_arms_to_top(self):
        for i in range(len(self.joint_state.position)):
            if self.joint_state.position[i] > -self.max_positions[i]:
                self.joint_state.position[i] += self.increment*2  

        if all(self.joint_state.position[i] <= -self.max_positions[i] for i in range(len(self.joint_state.position))):
            self.get_logger().info('Arms have reached the top.')
            self.moving_up = False  # Switch to shaking motion

    def shake_arms_top(self):
        for i in range(len(self.joint_state.position)):
            # If moving up
            self.joint_state.position[i] += self.increment*2
        # Log shake count
        if not self.joint_state.position[i] > -self.max_positions[i]:
            self.shake_count += 1  # Count the shake cycle
            self.get_logger().info(f'Shake count: {self.shake_count}')
            self.shake_up=False
     
    def shake_arms_down(self):
        for i in range(len(self.joint_state.position)):
            # If moving down
            self.joint_state.position[i] -= self.increment*2
        if not self.joint_state.position[i] < -self.min_positions[i]:
            self.shake_up=True


def main(args=None):
    rclpy.init(args=args)
    surprise_node = Surprise()
    
    try:
        rclpy.spin(surprise_node)
    except KeyboardInterrupt:
        pass

    surprise_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

