import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from std_msgs.msg import Bool

class FSMController(Node):
    def __init__(self):
            super().__init__('distance_emergency_stop')
            self.create_timer(0.1, self.run_loop)
            self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
            self.create_subscription(Bump, 'bump', self.process_bump, qos_profile_sensor_data)
            self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.range_lookup = {}
            self.state = "FINDING"
            self.scan = None
            self.kp_heading = -1*float(1/30)
            self.kp_driving = -6.0
            self.min_dist = None
            self.count = 0
            self.total_error = 0
            self.start_time = None
            self.follow_time = 10
            self.bump_state = False
            self.bump_started_at = None
            self.spiral_started_at = None
    
    def process_bump(self, msg):
        self.bump_state = bool(
            msg.left_front or msg.left_side or msg.right_front or msg.right_side
        )
        if self.bump_state and self.state != "BUMP":
            now = self.get_clock().now()
            if (self.bump_started_at is not None and
                    (now - self.bump_started_at).nanoseconds / 1e9 < 10):
                return
            self.bump_started_at = now
            self.state = "BUMP"
            self.drive()
            self.get_logger().info('Bumper contact: starting bump recovery')


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
        elif self.state == "BUMP":
            self.bump_protocol()
        elif self.state == "SPIRAL": 
            self.spiral()

    def bump_protocol(self):
        elapsed = (self.get_clock().now() - self.bump_started_at).nanoseconds / 1e9
        turn_end = 2 + math.pi / 0.6
        if elapsed < 2:
            self.drive(linear=0.2)
        elif elapsed < turn_end:
            self.drive(angular=-0.15)
        elif elapsed < turn_end + 2:
            self.drive(linear=0.2)
        else:
            self.drive()
            self.get_logger().info('Bump recovery finished')
            self.state = "SPIRAL"
            self.spiral_started_at = self.get_clock().now()
       
    def align(self, distance): 
        self.start_time = self.get_clock().now()
        msg = Twist()   
        dist_error = self.min_dist-distance
        angle_error= 90 -self.range_lookup[self.min_dist]
        msg.linear.x = 0.0                                                                                                                                                                                                                                                                 *self.kp_driving*dist_error
        msg.angular.z = self.kp_heading*angle_error 
        print('unfound', dist_error, angle_error, msg.angular.z)
        if abs(angle_error) < 2:
                self.state="FOUND"
        self.vel_pub.publish(msg)
    #+ 1*(angle_error + dist_error-self.total_error)
    def wall_follow(self, distance, speed):
        msg = Twist()
        angle_error = 90 -self.find_angle()
        dist_error = max(self.scan.ranges[self.find_angle()] - distance, -5)
        msg.angular.z = self.kp_heading*angle_error + .5*self.kp_driving*dist_error 
        self.total_error = dist_error+angle_error
        msg.linear.x = -0.5
        print(dist_error, angle_error, self.scan.ranges[self.find_angle()])
        self.vel_pub.publish(msg)

    
    def find_angle(self):   
        if self.scan: 
            closest = min(self.scan.ranges[45:135])
            idx = self.range_lookup[closest]
            return idx
        
    def spinning_follow(self, time): 
        msg = Twist()
        dist_error = self.min_dist
        angle_error = abs(90 -self.find_angle())
        
        msg.angular.z = -1*self.kp_heading*angle_error
        print(msg)
        self.vel_pub.publish(msg)    
    
    def process_scan(self, msg):
        self.scan = msg
        for i,j in enumerate(self.scan.ranges): 
            self.range_lookup[j] = i        
        self.min_dist = min(self.scan.ranges)

    def spiral(self):
        """Execute a short spiral motion
        """
        distance = 4*math.pi # arc length of short spiral
        time_driving = 10 # time = distance / velocity
        angular_vel = math.pi/(distance/.5) # angular velocity to complete the short spiral in the time_driving
        elapsed = (self.get_clock().now() - self.spiral_started_at).nanoseconds / 1e9
        if elapsed < time_driving:
            self.drive(linear=0.5, angular=angular_vel)
        else:
            self.drive()
            self.state = "FOUND"
    
    
    

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
