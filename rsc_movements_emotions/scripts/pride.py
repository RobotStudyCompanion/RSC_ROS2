#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Pride(Node):
    def __init__(self):
        super().__init__('pride')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the max and min positions for the joints
        self.max_positions = [0.7, -0.5]  
        self.min_positions = [-0.5, 0.7]  

        # Define the increment for smooth movement
        self.increment = -0.15

        self.shake_up = True
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 9    # Number of shakes to perform
        self.get_logger().info('Motion started.')
        
    def control_motion(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_up and self.shake_count<self.max_shakes:
            self.shake_arms_top()
        elif self.shake_count<self.max_shakes:
            self.shake_arms_down()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        self.publisher_.publish(self.joint_state)

    def shake_arms_top(self):
        if self.joint_state.position[0] > -self.max_positions[0]:
            self.joint_state.position[0] += self.increment
        if self.joint_state.position[1] < -self.max_positions[1]:
            self.joint_state.position[1] -= self.increment
        if self.joint_state.position[0] <= -self.max_positions[0]:
            self.shake_up=False
            self.shake_count+=1
     
    def shake_arms_down(self):
        if self.joint_state.position[0] < -self.min_positions[0]:
            self.joint_state.position[0] -= self.increment
        if self.joint_state.position[1] > -self.min_positions[1]:
            self.joint_state.position[1] += self.increment
        if self.joint_state.position[0] >= -self.min_positions[0]:
            self.shake_up=True



def main(args=None):
    rclpy.init(args=args)
    pride_node = Pride()
    
    try:
        rclpy.spin(pride_node)
    except KeyboardInterrupt:
        pass

    pride_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

