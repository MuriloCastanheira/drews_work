import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 
import tf_transformations
import math

class VelocityPublisher(Node):
    
    def __init__(self):
        super().__init__('velocity_publisher')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.go_to)
        self.get_logger().info('Velocity Publisher Node is running.')

        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Odometry subscriber node has been created')

        self.scan_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_callback, 10)
        self.scan_sub  # Prevent unused variable warning
        self.get_logger().info('LaserScan subscriber node has been created')

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.position = 0
        self.ranges = []

    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        
        r, p, self.yaw = tf_transformations.euler_from_quaternion(quaternion_list)
    
    def laser_callback(self, msg):
        self.ranges = msg.ranges
        
    def go_to(self):
        if not self.ranges:
            return

        twist_msg = Twist()
        
        goal_x = [10, 17.67951564671255]
        goal_y = [-6, 0.25332782243630697]
        diff_x = goal_x[self.position] - self.x
        diff_y = goal_y[self.position] - self.y

        distance = math.sqrt(diff_x ** 2 + diff_y ** 2)
        angle_to_goal = math.atan2(diff_y, diff_x)

        # Check the laser scan ranges for obstacles in front
        front_distance = min(self.ranges[50:155])  # Check a range of 90 degrees (135 to 225)
        
        if distance > 0.3:
            if front_distance < 0.3:  # If an obstacle is closer than 1 meter
                twist_msg.linear.x = -0.5
                twist_msg.angular.z = 50.0  # Turn to avoid the obstacle
                self.get_logger().info(f"Obstacle detected! Turning to avoid. Front distance: {front_distance}")
            else:
                twist_msg.linear.x = 1.0 * distance  # Scale velocity with distance
                twist_msg.angular.z = 0.7 * (angle_to_goal - self.yaw)
                self.get_logger().info(f"Reciv -->  x: {self.x}, Y: {self.y}, O: {self.yaw}")
                self.get_logger().info(f"Publi -->  x: {twist_msg.linear.x}, Y: {twist_msg.angular.z}")

            self.pub.publish(twist_msg)
        else:
            self.position = 1
            self.get_logger().info('Goal reached!')
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass

    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

