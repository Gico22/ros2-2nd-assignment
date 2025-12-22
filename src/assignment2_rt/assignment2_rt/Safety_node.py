import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from assignment2_rt_interfaces.msg import ObstacleInfo
from assignment2_rt_interfaces.srv import SetThreshold, GetAverages
import math


class Safety(Node):

    def __init__(self):
        super().__init__('Safety')

        # publishers
        self.obstacle_pub = self.create_publisher(ObstacleInfo, '/Obstacle_info', 10)
        self.safety_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # subscribers
        self.vel_sub = self.create_subscriber(Twist, '/cmd_vel', self.vel_read_callback, 10)
        self.scan_sub = self.create_subscriber(LaserScan, '/scan', self.scan_read_callback, 10)

        # variables
        self.safety_twist = Twist

        self.obstacle_info = ObstacleInfo()
        self.obstacle_info.distance = self.obstacle_info.direction = None
        self.threshold = self.obstacle_info.threshold = 1


    def vel_read_callback(self, vel):
        self.safety_twist.linear.x = -vel.linear.x
        self.safety_twist.angular.z = -vel.angular.z

    def scan_read_callback(self, scanner):
        # compute valid minimum obstacle distance
        valid_measurements = [
            (i, r) for i, r in enumerate(scanner.ranges)
            if scanner.range_min <= r <= scanner.range_max
        ]
        if not valid_measurements:
            return
        min_index, min_distance = min(valid_measurements, key=lambda x: x[1])
        self.obstacle_info.distance = min_distance

        # compute direction of the obstacle
        angle_i = scanner.angle_min + min_index * scanner.angle_increment
        if angle_i > math.pi/6:
            self.obstacle_info.direction = "Left"
        elif angle_i < -math.pi/6:
            self.obstacle_info.direction = "Right"
        else:
            self.obstacle_info.direction = "Front"  

def main(args=None):
    rclpy.init(args = args)
    Dist = Safety()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()