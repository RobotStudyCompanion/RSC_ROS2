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
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [1.8, 1.6, 0.2]  # Adjust these values based on your robot's limits
        self.min_positions = [1.6, 1.8, -0.2]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.2

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
            self.joint_state.position[0] += self.increment
        if self.joint_state.position[1] > -self.max_positions[1]:
            self.joint_state.position[1] += self.increment

        # Check if all joints have reached their maximum positions
        if self.joint_state.position[0] <= -self.max_positions[0] and self.joint_state.position[1] <= -self.max_positions[1]:
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

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment/4
        else: 
            self.joint_state.position[2]+=self.increment/4
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1
    pass


class Fun(Node):
    def __init__(self):
        super().__init__('fun')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 3.0, 0.8]  # Adjust these values based on your robot's limits
        self.min_positions = [3.0, 3.0, -0.8]
        # Define the increment for smooth movement
        self.increment = -0.5

        # State variables
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 50    # Number of shakes to perform
        self.dir=1
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_count<self.max_shakes:
            self.move_arms()
            self.shake_body()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

    def move_arms(self):
        self.joint_state.position[0] += self.increment
        self.joint_state.position[1] -= self.increment
        self.shake_count+=1
    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment/4
        else: 
            self.joint_state.position[2]+=self.increment/4
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1

    pass


class Caring(Node):
    def __init__(self):
        super().__init__('caring')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [0.7, -0.5, 0.4]  # Adjust these values based on your robot's limits
        self.min_positions = [-0.5, 0.7, -0.4]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.shake_up = True
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 4    # Number of shakes to perform
        self.dir=1
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_up and self.shake_count<self.max_shakes:
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

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment/4
        else: 
            self.joint_state.position[2]+=self.increment/4
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1
    pass

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
    pass

class Anger(Node):
    def __init__(self):
        super().__init__('anger')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0] 

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 2.2, 0.2]  
        self.min_positions = [2.2, 3.0, -0.2] 

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.moving_up = True  # True if moving towards the top position
        self.shake_up = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 4    # Number of shakes to perform
        self.dir=1
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.moving_up:
            # Perform shaking motion
            self.move_arms_to_top()
            #self.shake_body()
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

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment/2
        else: 
            self.joint_state.position[2]+=self.increment/2
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1


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
class Pride(Node):
    def __init__(self):
        super().__init__('pride')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [0.7, -0.5, 0.4]  # Adjust these values based on your robot's limits
        self.min_positions = [-0.5, 0.7, -0.4]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.shake_up = True
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 4    # Number of shakes to perform
        self.dir=1
        self.get_logger().info('Motion started.')

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_up and self.shake_count<self.max_shakes:
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

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment/4
        else: 
            self.joint_state.position[2]+=self.increment/4
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1
    pass

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
