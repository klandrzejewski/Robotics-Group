import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time
import csv

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90
STALL_TIME_THRESHOLD = 5  # Amount of seconds before detecting a stall

class WallWalker(Node):

    def __init__(self):
        super().__init__('wall_walker_node')
        self.scan_cleaned = []
        self.target_location = None
        self.stall = False
        self.recovery = False
        self.found_wall = False
        self.timer_start = time.time() # Start the timer
        self.timer_pos = None
        self.time_stationary = 0.0  # Time spent stationary
        self.last_move_time = time.time()  # Record the last move time
        self.time_last_wall = 0.0 # Time since last found wall
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        self.current_pos = None  # To store current position
        self.pose_saved = None  # Save last position for stall detection
        self.cmd = Twist()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        self.current_pos = msg2.pose.pose.position
        self.current_orientation = msg2.pose.pose.orientation

        # Check for stall
        if self.pose_saved is not None:
            diffX = math.fabs(self.pose_saved.x - self.current_pos.x)
            diffY = math.fabs(self.pose_saved.y - self.current_pos.y)
            
            # Update stationary time if robot has not moved significantly 
            if diffX < 0.00125 and diffY < 0.00125:
                current_time = time.time()
                self.time_stationary = current_time - self.last_move_time
                #self.last_move_time = current_time
            else:
                # Reset stall timer if the robot has moved
                self.time_stationary = 0.0
                self.last_move_time = time.time()

        self.pose_saved = self.current_pos  # Save current position for next comparison
        
        timer_val = time.time() - self.timer_start # Emergency stall
        if timer_val >= 60:
            if self.timer_pos is not None:
                timerDiffX = math.fabs(self.timer_pos.x - self.current_pos.x)
                timerDiffY = math.fabs(self.timer_pos.y - self.current_pos.y)
                if timerDiffX < 0.01 and timerDiffY < 0.01:
                    self.time_stationary = 6
            self.timer_pos = self.current_pos

    def timer_callback(self):
        if len(self.scan_cleaned) == 0 or self.current_pos is None:
            self.turtlebot_moving = False
            return
        
        current_time = time.time()

        # Get lidar readings
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        
        # Wall-following logic
        
        # Output position for logging
        # self.get_logger().info(f'Position: {self.current_pos}')

        # Check if it has found the wall recently
        if right_lidar_min > SAFE_STOP_DISTANCE + 0.35 and (current_time - self.time_last_wall > 5.0):
            self.found_wall = False
        
        # Check robot status
        if self.stall:
            self.cmd.linear.x = -0.5  # Reverse to recover from stall
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.stall = False
        elif self.recovery:
            self.cmd.linear.x = 0.0 
            if right_lidar_min > left_lidar_min: # Rotate to find a new path
                self.cmd.angular.z = -0.8  # Turn right
            else:
                self.cmd.angular.z = 0.8  # Turn left
            self.publisher_.publish(self.cmd)
            self.recovery = False
            self.time_stationary = 0.0
            self.last_move_time = time.time()
            self.timer_start = time.time()
        elif self.time_stationary >= STALL_TIME_THRESHOLD:
            self.cmd.linear.x = -0.5  # Reverse to recover from stall
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Stalled, recovering')
            self.time_stationary = 0.0
            self.last_move_time = time.time()
            self.stall = True  # Reset stall flag
            self.recovery = True # Set recovery flag
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            # Obstacle in front: slow down and turn
            self.cmd.linear.x = 0.07
            if right_lidar_min > SAFE_STOP_DISTANCE + 0.3 and self.found_wall == True: # Needs to turn, but follow the wall
                self.cmd.angular.z = -0.5  # Turn right
                self.time_last_wall = current_time
            else:
                self.cmd.angular.z = 0.5  # Turn left
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning to avoid front obstacle')
            self.turtlebot_moving = True
        else:
            # Space in front, follow the wall on the right side
            if right_lidar_min < SAFE_STOP_DISTANCE:
                # Robot is too close to the right wall, turn left slightly
                self.cmd.linear.x = 0.10
                self.cmd.angular.z = 0.1
                self.get_logger().info('Too close to wall, adjusting left')
                self.found_wall = True
                self.time_last_wall = current_time
            elif right_lidar_min > SAFE_STOP_DISTANCE + 0.2:
                # Robot is too far from the right wall, turn right slightly
                self.cmd.linear.x = 0.10
                self.cmd.angular.z = -0.18
                self.get_logger().info('Too far from wall, adjusting right')
                #self.found_wall == False
            else:
                # Distance is optimal, move forward
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.0
                self.get_logger().info('Following wall')
                self.found_wall = True
                self.time_last_wall = current_time

            # Publish the movement command
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True


def main(args=None):
    rclpy.init(args=args)
    room_explorer_node = WallWalker()
    rclpy.spin(room_explorer_node)
    room_explorer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()