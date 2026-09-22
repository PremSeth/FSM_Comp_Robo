""" 
Finite State Machine Controller for Neato Robot Navigation
"""

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
        """
        Initializes an FSM Controller Node that orchestrates different states for the navigation of a neato around a sim and real world environment
        """
        super().__init__('fsm')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10) #this node subscribes to laserscan because it needs the lidar data for wall follwong
        self.create_subscription(Bump, 'bump', self.process_bump, qos_profile_sensor_data) #this node subscribes to bump because it uses the bump to re align its position
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10) #publishes tot the cmd_vel topic to change robot speed and direction based on state
        self.range_lookup = {} #this is a dictionary that I initialize to help me figure out what angles are correlated to what distances, it helps with finding the angle with minimum distance
        self.state = "FINDING" #this is the initialized state of the robot wher eit finds the wall and aligns its wall following side or it's right side with the top being the front of the neato
        self.scan = None #this attribute holds the scanner message returned by the scan topic
        self.kp_heading = -1*float(1/30) #this is the proportion constant for when the robot turns to align and wall follow
        self.kp_dist = -3.0 #this constant is used to change the wall alignment by pulling neato closer in when dist > target_dist and opposite when to close
        self.min_dist = None #this attribute is to help quickly lookup what the distance value of the closest lidar scan is, ideally will be 90 when wall following
        self.bump_state = False #this initialzies the boolean of bump state to be false before the bumper gets hit at any point
        self.bump_started_at = None #this helps us figure out when the last bump started to time it 
        self.spiral_started_at = None # this helps us figure out when the spiral started to tiem its end
    
    def process_bump(self, msg):
        '''
        Callback function for neato message data from the bump topic
        '''
        
        self.bump_state = bool( 
            msg.left_front or msg.left_side or msg.right_front or msg.right_side
        ) #this boolean gets set to true when the bump is triggered
        if self.bump_state and self.state != "BUMP": #first makes sure that the true bump reading is not happening during the bump recovery cycle
            now = self.get_clock().now()
            if (self.bump_started_at is not None and
                    (now - self.bump_started_at).nanoseconds / 1e9 < 10): #if the last bump was less than ten seconds ago, ignore this bump as a false alarm
                return
            self.bump_started_at = now
            self.state = "BUMP"
            self.drive()
            self.get_logger().info('Bumper contact: starting bump recovery')


    def drive(self, linear=0.0, angular=0.0):
        '''
        Function to allow for simple neato driving commands with both linear and angular twist values
        '''
        
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.vel_pub.publish(vel) ## publishes the twist value to the vel topic which the neato uses for movement

    def run_loop(self):
        '''
        The final loop which orchestrates the neato by checking the state attribute
        '''
        
        if self.state == "FINDING" and self.scan: #if in the finding state and we have a scan value, we align to the wall
            self.align()
        elif self.state == "FOLLOW": # if in the follow state we wall follow
            self.wall_follow(.25)
        elif self.state == "BUMP": #if in the bump state we activate the recovery protocol
            self.bump_protocol()
        elif self.state == "SPIRAL": #if in the spiral state we do the huge spiral for its allocated time
            self.spiral()

    def bump_protocol(self):
        '''
        The bump protocol backs the neato out from a wall and reorients it to spiral
        '''
        
        elapsed = (self.get_clock().now() - self.bump_started_at).nanoseconds / 1e9 #this checks how long it has been doing the bump protocol for to allow for different sequences without sleeping
        turn_end = 2 + math.pi / 0.6
        if elapsed < 2:
            self.drive(linear=0.2) # drive forward
        elif elapsed < turn_end:
            self.drive(angular=-0.15) #turn right to point out before driving forward
        elif elapsed < turn_end + 1:
            self.drive(linear=1.0) #drive forward
        else:
            self.drive()
            self.get_logger().info('Bump recovery finished')
            self.state = "SPIRAL"
            print("Spiral state active")
            self.spiral_started_at = self.get_clock().now()
       
    def align(self): 
        '''
        Aligns the neato to its left side, assuming the top is the flat side of the neato
        '''
        msg = Twist()   
        angle_error = abs(90 -self.find_angle())
        # find angle gives us the angel at which we have the minimum distance, read its documentation to find out how
        # if we are going to be wall following along the left side it should be 90 which is why our error defines 90-the closest wall angle
        # I saw the assignment scaffoldign advised us to use two points and calibrate based off that, but I didn't really like how my error values came back for p-tuning 
        msg.linear.x = 0.0                                                                                                                 
        msg.angular.z = self.kp_heading*angle_error 
        print('unfound', angle_error, msg.angular.z)
        if abs(angle_error) < 2:
                self.state="FOLLOW" #once we have an angle error of less than 2 degrees, we switch to the wall following state
        self.vel_pub.publish(msg)
        
    def wall_follow(self, distance):
        print("wall following")
        '''
        Follows the wall along its left side and drives backward
        '''
        
        msg = Twist()
        angle_error = 90 -self.find_angle() #angle error is once again 90 the angle with minimum distance
        dist_error = max(self.scan.ranges[self.find_angle()] - distance, -5)
        # the distance error is the current minimum distance - the distance we want, and capped at -5, because sometimes the minimum distance becomes -inf
        # the reason we do it against the current minimum distance instead of where 90 is currently is because of 90 is very far off the following really wild and so by just 
        # making sure the minimum distance angle has a distance of what we want and then slowly changing the minimum angle to be 90 instead of what it currently is we 
        # get a smoother transition into the follow
        msg.angular.z = self.kp_heading*angle_error + self.kp_dist*dist_error #this uses a p value for both the angle error and the distance error and sums them to get the turning speed
        msg.linear.x = -0.5 # we drive forward at a constant speed, the reason this is constant is because doing a p controller on this made it really difficult
        # to tune, want to explore it further, but variable speed changes the correction a lot because we are correcting through driving in a direction
        print(dist_error, angle_error, self.find_angle()) #prints out error and what the minimum value is to help with debugging.
        self.vel_pub.publish(msg)

    
    def find_angle(self):   
        '''
        Returns the closest angle at any given moment when called
        '''
        if self.scan: #if we have lidar value continue
            closest = min(self.scan.ranges[45:135]) #sets closest to be the minimum distance in all of the range values
            idx = self.range_lookup[closest] #uses a lookup dictionary we initialize in processing to find what idx is correlated to this distance value
            return idx
   
    
    def process_scan(self, msg):
        '''
        On callback of the laserscan it updates a dictionary of distances that correlate to angles where they were taken
        '''
        self.scan = msg #sets the scan attribute to msg to make it accesibly outside of callback
        for i,j in enumerate(self.scan.ranges): #enumerate the scan values to give us both values and indexes which are angle on the lidar that have that value
            self.range_lookup[j] = i  #sets the range_lookup keys to be distances and the values to be indexes so we can get angles back given a distance value
        self.min_dist = min(self.scan.ranges) #keeps the minimum distance on standby if needed to look it up

    def spiral(self):
        """Execute a short spiral motion
        """
        distance = .2*math.pi # arc length of short spiral
        time_driving = 5 # time = distance / velocity
        angular_vel = math.pi/(distance/.5) # angular velocity to complete the short spiral in the time_driving
        elapsed = (self.get_clock().now() - self.spiral_started_at).nanoseconds / 1e9
        if elapsed < time_driving:
            self.drive(linear=0.5, angular=angular_vel)
        else:
            self.drive()
            self.state = "FOLLOW"    
    
    

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
