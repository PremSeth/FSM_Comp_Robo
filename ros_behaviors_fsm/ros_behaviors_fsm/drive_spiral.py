"""
Draw Spiral
--------
This node encapsulates implements a simple time-based approach to driving the
robot in a sprial shape.  The system makes use of a a special ``estop`` topic that
can trigger the robot to automatically stop when the value is true is received
on that topic.
"""
import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class DrawSpiral(Node):
    """A class that implements a node to pilot a robot in a square.
    """

    def __init__(self):
        super().__init__('draw_square_with_estop')
        self.e_stop = Event()
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()


    def handle_estop(self, msg): # will include distance E-stop too from Matthew
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """ 
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """Executes the main logic for driving the spiral.  This function does
        not return until the spiral is finished or the estop is pressed.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        while not self.e_stop.is_set():
            print("driving large spiral")
            self.l_spiral()
            print("driving short spiral")
            self.s_spiral()
        print('done with run loop')

    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s (.2 m/s for spiral)
            angular (_type_): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def l_spiral(self):
        """Execute spiral motion with a linear velocity of .2 m/s
        """
        distance = .2*math.pi # circumference of large semicircle
        time_driving = distance/.5 # time = distance / velocity

        angular_vel = math.pi/time_driving # angular velocity to complete the large semicircle in the time_driving
        if not self.e_stop.is_set():
            self.drive(linear=0.5, angular=angular_vel)
            sleep(time_driving)
            self.drive(linear=0.0, angular=0.0)

    def s_spiral(self):
        """Execute a short spiral motion
        """
        distance = .1*math.pi # arc length of short spiral
        time_driving = distance/.5 # time = distance / velocity
        angular_vel = math.pi/time_driving # angular velocity to complete the short spiral in the time_driving
        if not self.e_stop.is_set():
            self.drive(linear=0.5, angular=angular_vel)
            sleep(time_driving)
            self.drive(linear=0.0, angular=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSpiral()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
