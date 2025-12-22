import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from assignment2_rt_interfaces.srv import SetThreshold, GetAverages
import time

class UI(Node):

    def __init__(self):
        super().__init__('UI')

        # publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # variables
        self.twist = Twist()        # robot velocity (linear and angular)
        self.stop = Twist()         # robot stop command (twist with '0' values)

        # create timer
        self.timer = self.create_timer(0.1, self.user_interface) 

    def user_interface(self):
        # user input for selecting velocity and angular velocity
        try:
            x = float(input("Input linear velocity, range [-0.5, 0.5]: "))
            z = float(input("Input angular velocity, range [-1.0, 1.0]: "))
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            return
        
        self.twist.linear.x = self.clamp(x, -0.5, 0.5)
        self.twist.angular.z = self.clamp(z, -1.0, 1.0)

        # publish the velocity for 1 second
        self.vel_pub.publish(self.twist)
        time.sleep(5)
        self.vel_pub.publish(self.stop)
        self.twist = Twist()

    def clamp(self, val, vmin, vmax):
        return max(vmin, min(val, vmax))


def main(args=None):
    rclpy.init(args = args)
    Dist = UI()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()