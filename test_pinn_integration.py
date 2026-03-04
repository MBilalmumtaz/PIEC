#!/usr/bin/env python3
"""
Test script to verify PINN integration with path optimizer
"""
import rclpy
from rclpy.node import Node
from piec_pinn_surrogate_msgs.srv import EvaluateTrajectory
import numpy as np
import time
import math

class PINNTestClient(Node):
    def __init__(self):
        super().__init__('pinn_test_client')
        
        # Create client for PINN service
        self.cli = self.create_client(EvaluateTrajectory, '/evaluate_trajectory')
        
        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PINN service...')
        
        self.get_logger().info('✅ Connected to PINN service')
        
        # Test different path scenarios
        self.test_paths()
    
    def create_path(self, path_type):
        """Create different path types for testing"""
        if path_type == "straight":
            # Straight line
            xs = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
            ys = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            yaws = [0.0] * 6
            velocities = [1.0] * 6
            
        elif path_type == "curved":
            # Curved path
            xs = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
            ys = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
            yaws = [0.0, 0.5, 0.8, 1.0, 1.2, 1.4]
            velocities = [1.0] * 6
            
        elif path_type == "near_obstacle":
            # Path with obstacles nearby
            xs = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
            ys = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
            yaws = [0.0] * 6
            velocities = [0.5] * 6
            
        elif path_type == "zigzag":
            # Zigzag path (high instability)
            xs = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
            ys = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]
            yaws = [0.0, 0.8, -0.8, 0.8, -0.8, 0.0]
            velocities = [0.8] * 6
            
        return xs, ys, yaws, velocities
    
    def calculate_path_metrics(self, xs, ys):
        """Calculate path metrics for comparison"""
        # Path length
        length = 0
        for i in range(1, len(xs)):
            dx = xs[i] - xs[i-1]
            dy = ys[i] - ys[i-1]
            length += math.sqrt(dx*dx + dy*dy)
        
        # Curvature
        curvature = 0
        if len(xs) >= 3:
            for i in range(1, len(xs)-1):
                dx1 = xs[i] - xs[i-1]
                dy1 = ys[i] - ys[i-1]
                dx2 = xs[i+1] - xs[i]
                dy2 = ys[i+1] - ys[i]
                
                norm1 = math.sqrt(dx1*dx1 + dy1*dy1)
                norm2 = math.sqrt(dx2*dx2 + dy2*dy2)
                
                if norm1 > 0 and norm2 > 0:
                    dot = dx1*dx2 + dy1*dy2
                    cos_angle = dot / (norm1 * norm2)
                    cos_angle = max(-1.0, min(1.0, cos_angle))
                    curvature += abs(math.acos(cos_angle))
        
        avg_curvature = curvature / max(len(xs)-2, 1)
        
        return length, avg_curvature
    
    def test_paths(self):
        """Test different path types"""
        path_types = ["straight", "curved", "near_obstacle", "zigzag"]
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🔬 Testing PINN predictions for different paths")
        self.get_logger().info("="*60)
        
        for path_type in path_types:
            xs, ys, yaws, velocities = self.create_path(path_type)
            
            # Calculate request
            req = EvaluateTrajectory.Request()
            req.xs = xs
            req.ys = ys
            req.yaws = yaws
            req.velocities = velocities
            
            # Calculate path metrics
            length, curvature = self.calculate_path_metrics(xs, ys)
            
            # Call service
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result():
                energy = future.result().energy
                stability = future.result().stability
                
                self.get_logger().info(f"\n📌 Path type: {path_type.upper()}")
                self.get_logger().info(f"  Length: {length:.2f}m, Curvature: {curvature:.3f} rad")
                self.get_logger().info(f"  Energy: {energy:.2f} J")
                self.get_logger().info(f"  Stability: {stability:.3f}")
                
                # Provide interpretation
                if stability > 0.8:
                    self.get_logger().info("  ✅ HIGH STABILITY - Safe path")
                elif stability > 0.6:
                    self.get_logger().info("  ⚠️ MEDIUM STABILITY - Caution advised")
                else:
                    self.get_logger().info("  ❌ LOW STABILITY - Risky path")
            else:
                self.get_logger().error(f"Failed to get prediction for {path_type}")
            
            time.sleep(0.5)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✅ Test complete")
        
        # Shutdown
        rclpy.shutdown()

def main():
    rclpy.init()
    test_client = PINNTestClient()
    
if __name__ == '__main__':
    main()
