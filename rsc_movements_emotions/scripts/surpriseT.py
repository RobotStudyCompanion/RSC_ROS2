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
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 3.0, 0.4]  # Adjust these values based on your robot's limits
        self.min_positions = [2.2, 2.2, -0.4]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.3

        # State variables
        self.moving_up = True  # True if moving towards the top position
        self.shake_up = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 8    # Number of shakes to perform
        self.dir=1
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.moving_up:
            # Perform shaking motion
            self.move_arms_to_top()
        elif self.shake_up and self.shake_count<self.max_shakes:
            # Move arms to the top
            self.shake_arms_top()
            self.shake_body()
        elif self.shake_count<self.max_shakes:
            self.shake_arms_down()
            self.shake_body()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

    def move_arms_to_top(self):
        if self.joint_state.position[0] > -self.max_positions[0]:
            self.joint_state.position[0] += self.increment*2
        if self.joint_state.position[1] > -self.max_positions[1]:
            self.joint_state.position[1] += self.increment*2


        if self.joint_state.position[0] <= -self.max_positions[0] and self.joint_state.position[1] <= -self.max_positions[1]:        
            self.moving_up = False  # Switch to shaking motion

    def shake_arms_top(self):

        self.joint_state.position[0] += self.increment*2
        self.joint_state.position[1] += self.increment*2
        if not self.joint_state.position[0] > -self.max_positions[0]:
            self.shake_count += 1  # Count the shake cycle
            self.shake_up=False
     
    def shake_arms_down(self):

        self.joint_state.position[0] -= self.increment*2
        self.joint_state.position[1] -= self.increment*2
        if not self.joint_state.position[0] < -self.min_positions[0]:
            self.shake_up=True

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]+=0.08
        else: 
            self.joint_state.position[2]-=0.08
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1

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

