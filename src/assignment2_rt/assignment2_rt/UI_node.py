import rclpy
from rclpy.node import Node

class UI(Node):

    def __init__(self):
        super().__init__('UI')

def main(args=None):
    rclpy.init(args = args)
    Dist = UI()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()