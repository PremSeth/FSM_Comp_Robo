import rclpy
from rclpy.node import Node 
# Found from topic info in as type
from neato2_interfaces.msg import Bump  # local package call for a Bump type message format
from geometry_msgs.msg import Twist # CMD Vel
import math
from threading import Thread
from time import sleep


class EStopNode(Node):
    def __init__(self):
        super().__init__('e_stop_node')
        # Publisher to cmd vel to control neato speed
        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.bump_state = False
        # Subscription to Bump messages and callback function once a message is received
        self.sub = self.create_subscription(Bump, 'bump', self.process_bump, 10)
        # Initialize and run a thread that continously does the run loop
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

        

    # Callback function for when the neato is bumped
    def process_bump(self, msg):

        # Set the bump state to True if any part of the sensor is pressed
        self.bump_state = (msg.left_front == 1 or \
                           msg.right_front == 1 or \
                           msg.left_side == 1 or \
                           msg.right_side == 1) 

    # Drive function to send movement commands to Neato           
    def drive(self, linear=0.0, angular=0.0):
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.vel_pub.publish(vel)

    # Loop that runs through thread
    def run_loop(self): 
        while rclpy.ok():
            # Drive forward if Neato is not bumped
            if not self.bump_state:
                self.drive(linear=0.4)
                sleep(0.1)
                # Continue to not run next lines of code
                continue

            # Prints a log saying a bump has been detected
            self.get_logger().info('Bump detected')

            # Move the neato backwards
            self.drive(linear=-0.2)
            sleep(2)

            # Rotate neat 180 degrees
            self.drive(angular = 0.6)
            sleep(math.pi/0.6)

            # Set neato velocities back to 0 briefly
            self.drive()  # stop
            # Clear the bump boolean
            self.bump_state = False  
            # Log that the bump process is over 
            self.get_logger().info('Process over')
        
        

def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()            # cleanup
    

if __name__ == '__main__':
    main()
