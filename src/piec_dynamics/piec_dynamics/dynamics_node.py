#!/usr/bin/env python3
"""
ROS Node for Vehicle Dynamics Calculations
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from .vehicle_dynamics import VehicleDynamics
from .energy_model import PhysicsEnergyModel
from .traction_model import TractionModel
from .terrain_estimator import TerrainEstimator

class DynamicsNode(Node):
    def __init__(self):
        super().__init__('dynamics_node')
        
        # Declare parameters
        self.declare_parameter('robot_mass', 50.0)  # kg
        robot_mass = self.get_parameter('robot_mass').value
        
        # Initialize models with mass
        self.vehicle = VehicleDynamics()
        self.energy_model = PhysicsEnergyModel(self.vehicle)
        self.traction_model = TractionModel(mass=robot_mass)  # Pass mass here
        self.terrain_estimator = TerrainEstimator()
        
        # Subscribers
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Odometry, '/ukf/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # State variables
        self.current_velocity = 0.0
        self.current_acceleration = 0.0
        self.current_slope = 0.0
        self.current_roughness = 0.0
        self.terrain_type = 'asphalt'
        
        # Timer for periodic updates
        self.create_timer(0.1, self.update_dynamics)
        
        self.get_logger().info("Vehicle Dynamics Node started")
    
    def imu_callback(self, msg):
        """Process IMU data for terrain estimation"""
        imu_data = {
            'accel_x': msg.linear_acceleration.x,
            'accel_y': msg.linear_acceleration.y,
            'accel_z': msg.linear_acceleration.z,
            'orientation': msg.orientation
        }
        
        # Update terrain estimates
        slope, roughness, terrain_type = self.terrain_estimator.update(imu_data, None)
        self.current_slope = slope
        self.current_roughness = roughness
        self.terrain_type = terrain_type
    
    def odom_callback(self, msg):
        """Update velocity from UKF"""
        self.current_velocity = msg.twist.twist.linear.x
    
    def cmd_callback(self, msg):
        """Estimate acceleration from command"""
        # Simple acceleration estimation
        target_velocity = msg.linear.x
        acceleration = (target_velocity - self.current_velocity) / 0.1  # 0.1s time constant
        self.current_acceleration = acceleration
    
    def update_dynamics(self):
        """Periodic dynamics calculation"""
        # Calculate forces
        forces = self.vehicle.calculate_forces(
            self.current_velocity,
            self.current_acceleration,
            0,  # steering angle
            self.current_slope
        )
        
        # Calculate stability
        stability = self.traction_model.stability_metric(
            self.current_velocity,
            0,  # curvature
            self.current_slope,
            self.terrain_type
        )
        
        # Calculate power
        power = self.energy_model.calculate_instantaneous_power(
            self.current_velocity,
            self.current_acceleration,
            self.current_slope
        )
        
        # Log periodically
        self.get_logger().debug(
            f"Power: {power:.1f}W, Stability: {stability:.2f}, "
            f"Slope: {self.current_slope:.3f}rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
