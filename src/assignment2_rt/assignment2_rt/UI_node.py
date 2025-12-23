import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_rt_interfaces.srv import SetThreshold, GetAverages

class UI(Node):

    def __init__(self):
        super().__init__('UI')

        # publishers
        self.nodes_vel_pub = self.create_publisher(Twist, '/cmd_vel_user', 10)

        # services
        self.set_threshold_client = self.create_client(SetThreshold, '/set_threshold')
        self.get_averages_client = self.create_client(GetAverages, '/get_averages')
        while not self.set_threshold_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_threshold service...')
        while not self.get_averages_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_averages service...')

        # variables
        self.twist = Twist()        # robot velocity (linear and angular)
        self.linear_average = self.angular_average = 0  # average velocities values

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

        # sends the velocity to safety node, which will publish it to the robot
        self.nodes_vel_pub.publish(self.twist)
        self.twist = Twist()

        while True:
            user_decision = input("Select one of the following options:" \
            "'1': select robot velocities" \
            "'2': set a new threshold value for the robot" \
            "'3': get the average values of your last 5 inputs")
            if user_decision in {"1", "2", "3"}:
                break
            print("Invalid input")

        match user_decision:
            case "1":
                return
            case "2":
                req = SetThreshold.Request()
                req.threshold = float(input("Select the desired threshold (must be > 0)"))
                if req.threshold <= 0:
                    req.threshold = 0
                future = self.set_threshold_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                print(response.success, response.message)
            case "3":
                req = GetAverages.Request()
                future = self.get_averages_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.linear_average = response.linear_avg
                self.angular_average = response.angular_avg

            
    def clamp(self, val, vmin, vmax):
        return max(vmin, min(val, vmax))


def main(args=None):
    rclpy.init(args = args)
    Dist = UI()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()