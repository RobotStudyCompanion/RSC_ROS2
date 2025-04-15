#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Neutral(Node):
    def __init__(self):
        super().__init__('neutral')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body', 'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]

        # Define the min and max positions for the joints
        self.max_positions1 = [3.0, 3.0, 0.4] 
        self.min_positions1 = [0.0, 0.0, -0.4]  
        self.max_positions2 = [0.5, -0.5]  
        self.min_positions2 = [-0.5, 0.5]
        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.moving_up = True  # True if moving towards the top position
        self.moving_down = False
        self.shake_up = False
        self.shake_down = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 13    # Number of shakes to perform
        self.dir=1
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        if self.moving_up:
            self.move_arms_to_top()
            self.shake_body()
        elif self.moving_down:
            self.move_arms_to_bottom()
            self.shake_body()
        elif self.shake_up and self.shake_count<self.max_shakes:
            self.shake_arms_top()
            self.shake_body()
        elif self.shake_count<self.max_shakes:
            self.shake_arms_down()
            self.shake_body()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer
        self.publisher_.publish(self.joint_state)

    def move_arms_to_top(self):

        if self.joint_state.position[0] > -self.max_positions1[0]:
            self.joint_state.position[0] += self.increment*3
        if self.joint_state.position[1] > -self.max_positions1[1]:
            self.joint_state.position[1] += self.increment*3

        if self.joint_state.position[0] <= -self.max_positions1[0] and self.joint_state.position[1] <= -self.max_positions1[1]: 
            self.get_logger().info('Arms have reached the top.')
            self.moving_up = False 
            self.moving_down = True

    def move_arms_to_bottom(self):
        if self.joint_state.position[0] < -self.min_positions1[0]:
            self.joint_state.position[0] -= self.increment*3
        if self.joint_state.position[1] < -self.min_positions1[1]:
            self.joint_state.position[1] -= self.increment*3

        if self.joint_state.position[0] >= -self.min_positions1[0] and self.joint_state.position[1] >= -self.min_positions1[1]: 
            self.get_logger().info('Arms have reached the bottom.')
            self.moving_down = False 
            self.shake_up = True

    def shake_arms_top(self):
        if self.joint_state.position[0] > -self.max_positions2[0]:
            self.joint_state.position[0] += self.increment
        if self.joint_state.position[1] < -self.max_positions2[1]:
            self.joint_state.position[1] -= self.increment
        if self.joint_state.position[0] <= -self.max_positions2[0]:
            self.shake_up=False
            self.shake_count+=1
     
    def shake_arms_down(self):
        if self.joint_state.position[0] < -self.min_positions2[0]:
            self.joint_state.position[0] -= self.increment
        if self.joint_state.position[1] > -self.min_positions2[1]:
            self.joint_state.position[1] += self.increment
        if self.joint_state.position[0] >= -self.min_positions2[0]:
            self.shake_up=True

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment/4
        else: 
            self.joint_state.position[2]+=self.increment/4
        if self.joint_state.position[2]>=self.max_positions1[2] or self.joint_state.position[2]<=self.min_positions1[2]:
            self.dir = self.dir*-1

def main(args=None):
    rclpy.init(args=args)
    joy_node = Neutral()
    
    try:
        rclpy.spin(joy_node)
    except KeyboardInterrupt:
        pass

    joy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

