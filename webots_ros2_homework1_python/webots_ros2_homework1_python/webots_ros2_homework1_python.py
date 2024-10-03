import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
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
        timer_period = 0.5
        self.pose_saved = None
        self.start_yaw = None
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.total_distance = 0
        self.last_saved = None
        self.start = None


    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
       
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)



    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));

        self.pose_saved=position

        # Convert quaternion to yaw
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        self.current_yaw = yaw

        # Initialize
        if self.start is None:
            self.start = self.pose_saved

        if self.last_saved is None:
            self.last_saved = position
        
        if self.start_yaw is None:
            self.start_yaw = yaw
        
           
        return None
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        
        # Check if it has reached it's goal
        # Compute the yaw difference between the current orientation and starting orientation
        yaw_diff = self.current_yaw - self.start_yaw

        # Normalize the yaw difference to the range [-pi, pi]
        #yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))

        # 10 degrees = 0.1745 radians
        # 180 degrees = 3.14159 radians
        self.get_logger().info('Yaw difference: {} radians'.format(yaw_diff))
        if abs(yaw_diff) >= 3.14159:  # If the robot has rotated by trial degrees
            self.cmd.angular.z = 0.0  # Stop rotating
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Yaw difference: {} radians'.format(yaw_diff))
        # if self.pose_saved.x >= (self.start.x + 1): 
        #     self.cmd.linear.x = 0.0 # Stop moving
        #     self.cmd.linear.z = 0.0
        #     self.cmd.angular.z = 0.0
        #     self.publisher_.publish(self.cmd)

        #     current = math.sqrt((self.pose_saved.x - self.last_saved.x)** 2 + (self.pose_saved.y - self.last_saved.y)**2) # Get final distance
        #     self.last_saved = self.pose_saved
        #     self.total_distance = self.total_distance + current
        #     self.get_logger().info('Estimated Distance: "%s"' % (self.pose_saved.x - self.start.x))
        #     self.get_logger().info('Actual Distance: "%s"' % self.total_distance)
        #     self.total_distance = 0 # Reset for next trial
        #     #self.start = self.pose_saved
        else:
            #self.cmd.linear.x = 0.075 # Move forward at trial speed
            #self.cmd.linear.x = 0.150 # Move forward at trial speed
            self.cmd.linear.x = 0.0 
            #self.cmd.linear.z = 0.0
            #self.cmd.angular.z = 0.5236 # Turn at trial angle (30)
            self.cmd.angular.z = 2.094 # Turn at trial angle (120)
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True

            current = math.sqrt((self.pose_saved.x - self.last_saved.x)** 2 + (self.pose_saved.y - self.last_saved.y)**2) # Calculate actual distance
            self.last_saved = self.pose_saved # Reset last saved
            self.total_distance = self.total_distance + current # Add to total distance
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    