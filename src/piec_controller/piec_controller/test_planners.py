#!/usr/bin/env python3
"""
Test script to verify planners
"""
import rclpy
from rclpy.node import Node

class PlannerTester(Node):
    def __init__(self):
        super().__init__('planner_tester')
        
        # Set parameters to disable PINN
        self.set_parameters([
            self.create_parameter('use_pinn_predictions', False),
            self.create_parameter('use_pinn', False)
        ])
        
        self.get_logger().info("✅ PINN disabled. Restart your nodes.")
        
        # Exit after setting parameters
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PlannerTester()

if __name__ == '__main__':
    main()
