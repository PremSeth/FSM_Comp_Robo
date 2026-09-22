import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from threading import Thread
from time import sleep
from neato2_interfaces.msg import Bump
from std_msgs.msg import Bool

class FSMController(Node):
    def __init__(self):
            super().__init__('distance_emergency_stop')
            self.create_timer(0.1, self.run_loop)
            self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
            self.create_subscription(Bump, 'bump', self.process_bump, 10)  
            self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.range_lookup = {}
            self.state = "FINDING"
            self.scan = None
            self.kp_heading = -1*float(1/30)
            self.kp_driving = 1.0
            self.min_dist = None
            self.count = 0
            self.total_error = 0
            self.start_time = None
            self.follow_time = 10
            self.bump_state = False
    
    def process_bump(self, msg):

        # Set the bump state to True if any part of the sensor is pressed
        self.bump_state = (msg.left_front == 1 or \
                           msg.right_front == 1 or \
                           msg.left_side == 1 or \
                           msg.right_side == 1) 
        if self.bump_state: 
            self.state = "BUMP"
            self.bump_state = False

    def drive(self, linear=0.0, angular=0.0):
            vel = Twist()
            vel.linear.x = linear
            vel.angular.z = angular
            self.vel_pub.publish(vel)

    def run_loop(self):
        if self.state == "FINDING" and self.scan: 
            self.align(.25)
        elif self.state == "FOUND": 
            self.wall_follow(.25, 1)
            if (self.get_clock().now() - self.start_time).nanoseconds/1e9 > 10: 
                self.state = "BUMP"
        elif self.state == "BUMP":
            self.bump_protocol()
        elif self.state == "SPIRAL": 
            self.s_spiral()

    def bump_protocol(self):
        self.get_logger().info('Bump detected')

        # Move the neato backwards
        self.drive(linear=-0.2)
        sleep(2)

        # Rotate neat 180 degrees
        self.drive(angular = 0.628)
        sleep(.125)

        # Set neato velocities back to 0 briefly
        self.drive()  # stop
        # Clear the bump booleanq
        self.bump_state = False
        # Log that the bump process is over 
        self.get_logger().info('Process over')
        self.state = "SPIRAL"
       
    def align(self, distance): 
        self.start_time = self.get_clock().now()
        msg = Twist()
        dist_error = self.min_dist-distance
        angle_error1 = 90 -self.range_lookup[self.min_dist]
        angle_error2 = self.range_lookup[self.min_dist]-360
        if abs(angle_error1) < abs(angle_error2): 
            angle_error = angle_error1
        else: 
            angle_error = angle_error2
        msg.linear.x = 0.0                                                                                                                                                                                                                                                                 *self.kp_driving*dist_error
        msg.angular.z = self.kp_heading*angle_error 
        print('unfound', dist_error, angle_error)
        if abs(angle_error) < 2:
                self.state="FOUND"
        self.vel_pub.publish(msg)
    
    def wall_follow(self, distance, speed):
        msg = Twist()
        angle_error = 90 -self.find_angle()
        dist_error = max(self.scan.ranges[self.find_angle()] - distance, -5)
        msg.angular.z = self.kp_heading*angle_error - 6*dist_error - .1*(angle_error + dist_error-self.total_error)
        self.total_error = dist_error+angle_error
        msg.linear.x = -0.5
        print(dist_error, angle_error)
        self.vel_pub.publish(msg)

    
    def find_angle(self):   
        if self.scan: 
            closest = min(self.scan.ranges[45:135])
            idx = self.range_lookup[closest]
            return idx
        
    def spinning_follow(self, time): 
        msg = Twist()
        dist_error = self.min_dist
        angle_error = 90 -self.find_angle()
        
        msg.angular.z = -1*self.kp_heading*angle_error
        print(msg)
        self.vel_pub.publish(msg)    
    
    def process_scan(self, msg):
        self.scan = msg
        for i,j in enumerate(self.scan.ranges): 
            self.range_lookup[j] = i        
        self.min_dist = min(self.scan.ranges)

    def s_spiral(self):
        """Execute a short spiral motion
        """
        distance = 2*math.pi # arc length of short spiral
        time_driving = distance/.2 # time = distance / velocity
        angular_vel = math.pi/time_driving # angular velocity to complete the short spiral in the time_driving
        if True:
            self.drive(linear=0.2, angular=angular_vel)
            sleep(time_driving)
            self.drive(linear=0.0, angular=0.0)
    
    
    

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()