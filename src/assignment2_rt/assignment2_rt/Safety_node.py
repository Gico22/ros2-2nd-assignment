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
        self.obstacle_pub = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # subscribers
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel_user', self.vel_read_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_read_callback, 10)

        # services
        self.set_threshold_srv = self.create_service(SetThreshold, '/set_threshold', self.set_threshold_callback)
        self.get_averages_srv = self.create_service(GetAverages, '/get_averages', self.get_averages_callback)

        # variables
        self.user_twist = Twist()
        self.safety_twist = Twist()
        self.stop = Twist()
        self.scan_received = False
        self.safe = True
        self.vel_history= []

        self.obstacle_info = ObstacleInfo()
        self.obstacle_info.distance = self.obstacle_info.direction = None
        self.threshold = self.obstacle_info.threshold = 1   # default threshold

        # create timer
        self.timer = self.create_timer(0.1, self.safety_controller) 

    def safety_controller(self):
        if not self.scan_received:
            return
        
        # publish the obstacle informations
        self.obstacle_pub.publish(self.obstacle_info)

        # publishes the user velocities if the robot hasn't passed the threshold
        if self.safe:
            self.vel_pub.publish(self.user_twist)

        # publish inverse velocities if threshold is passed 
        if self.obstacle_info.distance <= self.threshold:
            self.safe = False
            self.vel_pub.publish(self.safety_twist)
        
        # stop the robot after it reaches the safe zone
        if not self.safe and self.obstacle_info.distance > (self.threshold + 0.1):
            self.safe = True
            self.vel_pub.publish(self.stop)

    def vel_read_callback(self, vel):
        # publishes the user velocities
        self.user_twist.linear.x = vel.linear.x
        self.user_twist.angular.z = vel.angular.z

        # updates safety twist
        self.safety_twist.linear.x = -vel.linear.x
        self.safety_twist.angular.z = -vel.angular.z

        # memorize the last 5 user inputs
        self.vel_history.append((vel.linear.x, vel.angular.z))
        if len(self.vel_history) > 5:
            self.vel_history.pop(0)

    def scan_read_callback(self, scanner):
        self.scan_received = True
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

    def set_threshold_callback(self, request, response):
        if request.threshold <= 0:
            self.threshold = self.obstacle_info.threshold = 0.1
            response.success = False
            response.message = f"Input threshold too low, threshold set to 0.1"
        else:
            self.threshold = self.obstacle_info.threshold = request.threshold
            response.success = True
            response.message = f"Threshold set to {self.threshold}"
        return response
    
    def get_averages_callback(self, request, response):
        if len(self.vel_history) == 0:
            response.linear_avg = response.angular_avg = 0
            return response
        response.linear_avg = sum(v for v, _ in self.vel_history) / len(self.vel_history)
        response.angular_avg = sum(w for _, w in self.vel_history) / len(self.vel_history)
        return response

def main(args=None):
    rclpy.init(args = args)
    Dist = Safety()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()