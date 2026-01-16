#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main():
    print("Initializing rclpy...")
    rclpy.init()
    print("Creating node...")
    node = Node('test_node')
    print(f"Node created: {node.get_name()}")
    print("Spinning...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
