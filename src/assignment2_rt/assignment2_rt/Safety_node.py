import rclpy
from rclpy.node import Node

class Safety(Node):

    def __init__(self):
        super().__init__('Safety')

def main(args=None):
    rclpy.init(args = args)
    Dist = Safety()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()