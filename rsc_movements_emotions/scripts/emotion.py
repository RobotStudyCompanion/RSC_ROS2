#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Joy(Node):
    def __init__(self):
        super().__init__('joy')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body']
        self.joint_state.position = [0.0, 0.0] 

        # Define the min and max positions for the joints
        self.max_positions = [1.8, 1.6]  
        self.min_positions = [1.6, 1.8] 

        # Define the increment for smooth movement
        self.increment = -0.2

        self.moving_up = True  # True if moving towards the top position
        self.shake_up = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 35    # Number of shakes to perform
        self.get_logger().info('Motion started.')

    def control_motion(self):
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
        if self.joint_state.position[0] > -self.max_positions[0]:
            self.joint_state.position[0] += self.increment
        if self.joint_state.position[1] > -self.max_positions[1]:
            self.joint_state.position[1] += self.increment


        # Check if all joints have reached their maximum positions
        if all(self.joint_state.position[i] <= -self.max_positions[i] for i in range(len(self.joint_state.position))):
            self.moving_up = False  # Switch to shaking motion
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
    pass


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
    pass


class Caring(Node):
    def __init__(self):
        super().__init__('caring')
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
    pass

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
    pass

class Anger(Node):
    def __init__(self):
        super().__init__('anger')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 hz
        self.joint_state = JointState()

        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the max and min positions for the joints
        self.max_positions = [3.0, 2.2] 
        self.min_positions = [2.2, 3.0] 

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.moving_up = True  # True if moving towards the top position
        self.shake_up = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 13    # Number of shakes to perform
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
            self.timer.cancel() 

        self.publisher_.publish(self.joint_state)

    def move_arms_to_top(self):

        if self.joint_state.position[0] > -self.max_positions[0]:
            self.joint_state.position[0] += self.increment*2
        if self.joint_state.position[1] > -self.max_positions[1]:
            self.joint_state.position[1] += self.increment*2
        if self.joint_state.position[0] <= -self.max_positions[0] and self.joint_state.position[1] <= -self.max_positions[1]: 
            self.moving_up = False 

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
    pass

class Neutral(Node):
    def __init__(self):
        super().__init__('neutral')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body']
        self.joint_state.position = [0.0, 0.0]

        # Define the min and max positions for the joints
        self.max_positions1 = [3.0, 3.0] 
        self.min_positions1 = [0.0, 0.0]  
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
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        if self.moving_up:
            self.move_arms_to_top()
        elif self.moving_down:
            self.move_arms_to_bottom()
        elif self.shake_up and self.shake_count<self.max_shakes:
            self.shake_arms_top()
        elif self.shake_count<self.max_shakes:
            self.shake_arms_down()
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
    pass

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
    pass

def main():
    nodes = {
        "1": Joy,
        "2": Fun,
        "3": Caring,
        "4": Pride,  
        "5": Anger,
        "6": Surprise,
        "7": Neutral
    }

    print("Choose a node to run:")
    print("1: Joy (Node 1)")
    print("2: Fun (Node 2)")
    print("3: Caring (Node 3)")
    print("4: Pride (Node 4)")
    print("5: Anger (Node 5)")
    print("6: Surprise (Node 6)")
    print("7: Neutral (Node 7)")

    choice = input("Enter your choice (1-7): ").strip()

    if choice in nodes:
        rclpy.init()
        try:
            node = nodes[choice]()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("Invalid choice. Exiting.")


if __name__ == '__main__':

        main()
