import rclpy
from rclpy.node import Node


class Placeholder(Node):
    def __init__(self):
        super().__init__("placeholder")


def main():
    rclpy.init()
    rclpy.spin(Placeholder())
    rclpy.shutdown()
