import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time

class RandomWalk(Node):

    def __init__(self):
        super().__init__('random_walk_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.odom_data = None
        self.initial_pose = None
        self.cmd = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.test_running = False
        self.test_phase = 0
        self.movement_started_time = None
        self.move_distance = 0
        self.target_distance = 1.0  # Default to 1 meter
        self.target_angle = 0.0  # Default no rotation
        self.speed = 0.075  # Default speed
        self.rotation_speed = 0.52  # Default angular speed
        self.trial_count = 0
        self.max_trials = 5

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.odom_data = (position.x, position.y, orientation.z)

    def start_movement(self, linear, angular, duration):
        self.cmd.linear.x = linear
        self.cmd.angular.z = angular
        self.publisher_.publish(self.cmd)
        self.movement_started_time = time.time()
        self.test_running = True
        self.get_logger().info(f"Started movement: linear={linear}, angular={angular}, duration={duration}s")

    def stop_movement(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.test_running = False

    def calculate_distance(self, start_pose, end_pose):
        #Calculate linear distance between two positions."""
        return math.sqrt((end_pose[0] - start_pose[0]) ** 2 + (end_pose[1] - start_pose[1]) ** 2)

    def calculate_angle(self, start_orientation, end_orientation):
        #Calculate angular difference."""
        return abs(end_orientation - start_orientation)

    def run_trial(self, distance, linear_speed, rotation_angle, rotation_speed):
        # Run one trial of moving a specific distance or rotating by a specific angle."""
        if distance > 0:
            # Move forward
            self.get_logger().info(f"Starting trial for distance: {distance}m at speed {linear_speed}m/s")
            self.start_movement(linear_speed, 0.0, distance / linear_speed)
        elif rotation_angle > 0:
            # Rotate
            self.get_logger().info(f"Starting trial for rotation: {rotation_angle} degrees at speed {rotation_speed} rad/s")
            self.start_movement(0.0, rotation_speed, math.radians(rotation_angle) / rotation_speed)
        self.initial_pose = self.odom_data

    def timer_callback(self):
        # Run the test sequence and log odometry error."""
        if not self.test_running:
            if self.trial_count < self.max_trials:
                # Alternate between linear and rotational trials
                if self.test_phase == 0:
                    self.run_trial(self.target_distance, self.speed, 0, 0)
                    self.test_phase = 1
                elif self.test_phase == 1:
                    self.run_trial(0, 0, 180, self.rotation_speed)
                    self.test_phase = 0
                self.trial_count += 1
            else:
                self.get_logger().info("Trials completed.")
        else:
            # Stop movement after the duration has passed
            if time.time() - self.movement_started_time > self.target_distance / self.speed:
                self.stop_movement()
                actual_distance = self.calculate_distance(self.initial_pose, self.odom_data)
                self.get_logger().info(f"Trial completed. Actual distance: {actual_distance}, Odometry reported: {self.target_distance}")

def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    rclpy.spin(random_walk_node)
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()