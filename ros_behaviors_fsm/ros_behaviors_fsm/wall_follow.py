import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DistanceEmergencyStopNode(Node):
    def __init__(self):
        super().__init__('distance_emergency_stop')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.range_lookup = {}
        self.distance_to_obstacle = 0.0
        self.Kp = 0.4
        self.target_distance = 0.25
        self.state = "FINDING"
        self.scan = None
        self.kp_heading = float(1/30)
        self.kp_driving = 1.0
        self.min_dist = None
        self.count = 0
        self.total_error = 0
        self.start_time = None
        self.follow_time = 10
    
    def run_loop(self):
        if self.state == "FINDING" and self.scan: 
            self.align(self.target_distance)
        if self.state == "FOUND": 
            self.wall_follow(self.target_distance, 1)
            
    def align(self, distance): 
        msg = Twist()
        dist_error = self.min_dist-distance
        angle_error = 0 -self.range_lookup[self.min_dist]
        
        msg.linear.x = 0.0                                                                                                                                                                                                                                                                 *self.kp_driving*dist_error
        msg.angular.z = -1*self.kp_heading*angle_error 
        print('unfound', dist_error, angle_error)
        if abs(angle_error) < 2:
                self.state="FOUND"
        self.vel_pub.publish(msg)
    
    def wall_follow(self, distance, speed):
        msg = Twist()
        angle_error = 90 -self.find_angle()
        dist_error = max(self.scan.ranges[self.find_angle()] - distance, -5)
        msg.angular.z = -1*self.kp_heading*angle_error - 6*dist_error - .1*(angle_error + dist_error-self.total_error)
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


def main(args=None):
    rclpy.init(args=args)
    node = DistanceEmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
