#!/usr/bin/env python3
"""
Enhanced PINN Service Node - With Actual PhysicsInformedPINN Model
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import torch
import torch.nn as nn
import os
import math
import time
from concurrent.futures import ThreadPoolExecutor
import threading
import sys

# Add the directory containing pinn_model.py to Python path
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import your actual PINN model
try:
    from pinn_model import PhysicsInformedPINN, load_model
    PINN_MODEL_AVAILABLE = True
    print("✅ Successfully imported PhysicsInformedPINN")
except ImportError as e:
    PINN_MODEL_AVAILABLE = False
    print(f"⚠️ Could not import PhysicsInformedPINN: {e}")
    print("Will use fallback model")

try:
    from piec_pinn_surrogate_msgs.srv import EvaluateTrajectory
except ImportError:
    # Fallback for development
    print("⚠️ piec_pinn_surrogate_msgs not found, using fallback")
    class EvaluateTrajectory:
        class Request:
            def __init__(self):
                self.xs = []
                self.ys = []
                self.yaws = []
                self.velocities = []
        
        class Response:
            def __init__(self):
                self.energy = 0.0
                self.stability = 0.0

class PINNService(Node):
    def __init__(self):
        super().__init__('pinn_service')
        
        service_name = '/evaluate_trajectory'
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'PINN Service starting: {service_name}')
        self.get_logger().info('=' * 60)
        
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('stability_scale', 1.5)
        self.declare_parameter('energy_scale', 1.0)
        self.declare_parameter('diagnostic_mode', True)  # NEW: Enable diagnostic logging
        
        self.debug_mode = self.get_parameter('debug_mode').value
        self.model_path = self.get_parameter('model_path').value
        self.use_gpu = self.get_parameter('use_gpu').value
        self.stability_scale = self.get_parameter('stability_scale').value
        self.energy_scale = self.get_parameter('energy_scale').value
        self.diagnostic_mode = self.get_parameter('diagnostic_mode').value

        # Thread pool
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        self.request_lock = threading.Lock()
        
        # Create service
        self.srv = self.create_service(
            EvaluateTrajectory,
            service_name,
            self.evaluate_callback,
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        
        # Initialize model
        self.model = None
        self.scaler = None
        self.device = torch.device('cuda' if torch.cuda.is_available() and self.use_gpu else 'cpu')
        
        self.get_logger().info(f"Using device: {self.device}")
        self.get_logger().info(f"CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            self.get_logger().info(f"GPU: {torch.cuda.get_device_name(0)}")
        
        # Load model
        self.load_model()
        
        # Statistics
        self.request_count = 0
        self.success_count = 0
        self.avg_response_time = 0.0
        self.response_times = []
        
        self.get_logger().info(f'✅ PINN Service READY at: {service_name}')
        self.get_logger().info(f'Model loaded: {self.model is not None}')
        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info(f'Thread pool: {self.thread_pool._max_workers} workers')
        self.get_logger().info(f'Diagnostic mode: {self.diagnostic_mode}')
    
    def load_model(self):
        """Load the actual PhysicsInformedPINN model"""
        try:
            # Try multiple possible model paths
            model_paths = []
            if self.model_path:
                model_paths.append(self.model_path)
            model_paths += [
                '/home/amjad/PIEC_2d/src/piec_pinn_surrogate/models/pinn_physics.pt',
                '/home/amjad/PIEC_2d/install/piec_pinn_surrogate/share/piec_pinn_surrogate/models/pinn_physics.pt',
                os.path.join(os.path.dirname(__file__), '..', 'models', 'pinn_physics.pt'),
                os.path.join(os.path.dirname(__file__), 'models', 'pinn_physics.pt'),
                'pinn_physics.pt'
            ]
            
            for model_path in model_paths:
                if os.path.exists(model_path):
                    self.get_logger().info(f'Loading PINN model from: {model_path}')
                    
                    # Load checkpoint
                    checkpoint = torch.load(model_path, map_location=self.device)
                    
                    # Get model parameters
                    input_dim = checkpoint.get('input_dim', 10)
                    hidden_dim = checkpoint.get('hidden_dim', 128)
                    
                    self.get_logger().info(f'Model architecture: input_dim={input_dim}, hidden_dim={hidden_dim}')
                    
                    # Create the actual PhysicsInformedPINN model
                    if PINN_MODEL_AVAILABLE:
                        self.model = PhysicsInformedPINN(
                            input_dim=input_dim,
                            hidden_dim=hidden_dim
                        ).to(self.device)
                        self.get_logger().info('Created PhysicsInformedPINN instance')
                    else:
                        # Create a compatible fallback model with the same architecture
                        self.model = self.create_compatible_model(input_dim, hidden_dim).to(self.device)
                        self.get_logger().info('Created compatible fallback model')
                    
                    # Load the state dict - FIXED: Use 'model_state' key
                    if 'model_state' in checkpoint:
                        state_dict = checkpoint['model_state']
                        self.get_logger().info('Found model_state key in checkpoint')
                    else:
                        state_dict = checkpoint
                        self.get_logger().info('Using checkpoint as state dict')
                    
                    # Try to load with strict=False to see which keys mismatch
                    try:
                        self.model.load_state_dict(state_dict, strict=True)
                        self.get_logger().info('✅ Model loaded with strict=True')
                    except Exception as e:
                        self.get_logger().warn(f'Strict loading failed: {e}')
                        self.get_logger().info('Attempting to load with strict=False...')
                        self.model.load_state_dict(state_dict, strict=False)
                        self.get_logger().info('✅ Model loaded with strict=False')
                    
                    # Load scaler if available
                    self.scaler = checkpoint.get('scaler', None)
                    if self.scaler:
                        self.get_logger().info('Scaler loaded successfully')
                        if 'Y_mean' in self.scaler:
                            self.get_logger().info(f"Y_mean: {self.scaler['Y_mean']}")
                            self.get_logger().info(f"Y_std: {self.scaler['Y_std']}")
                    
                    # Set to evaluation mode
                    self.model.eval()
                    
                    # Print model info
                    if 'features' in checkpoint:
                        self.get_logger().info(f"Features: {checkpoint['features']}")
                    
                    # Warm up the model
                    self.warmup_model()
                    
                    self.get_logger().info(f'✅ PINN model loaded successfully on {self.device}')
                    return
            
            self.get_logger().warn('PINN model not found, using physics-based predictions only')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load PINN model: {e}')
            import traceback
            traceback.print_exc()
    
    def create_compatible_model(self, input_dim, hidden_dim):
        """Create a model compatible with the saved state dict"""
        class CompatiblePINN(nn.Module):
            def __init__(self, input_dim, hidden_dim):
                super().__init__()
                self.fc1 = nn.Linear(input_dim, hidden_dim)
                self.fc2 = nn.Linear(hidden_dim, hidden_dim)
                self.fc3 = nn.Linear(hidden_dim, hidden_dim // 2)
                self.fc4 = nn.Linear(hidden_dim // 2, 2)
                self.activation = nn.Tanh()
                self.dropout = nn.Dropout(0.1)
            
            def forward(self, x):
                h1 = self.activation(self.fc1(x))
                h1 = self.dropout(h1)
                h2 = self.activation(self.fc2(h1))
                h2 = self.dropout(h2)
                h3 = self.activation(self.fc3(h2))
                return self.fc4(h3)
        
        return CompatiblePINN(input_dim, hidden_dim)
    
    def warmup_model(self):
        """Warm up the model with dummy inference"""
        if self.model is None:
            return
        
        try:
            # Create dummy input (batch_size=1, input_dim=10)
            dummy_input = torch.randn(1, 10).to(self.device)
            
            # Run inference multiple times to warm up
            with torch.no_grad():
                for _ in range(3):
                    _ = self.model(dummy_input)
            
            # Synchronize if using CUDA
            if self.device.type == 'cuda':
                torch.cuda.synchronize()
            
            self.get_logger().debug("Model warmed up successfully")
        except Exception as e:
            self.get_logger().debug(f"Warmup failed (non-critical): {e}")
    
    def extract_features(self, xs, ys, yaws, velocities):
        """Extract 10 features for the PINN model"""
        n = len(xs)
        if n < 2:
            return np.zeros(10, dtype=np.float32)
        
        # Convert to numpy arrays
        xs = np.array(xs, dtype=np.float32)
        ys = np.array(ys, dtype=np.float32)
        yaws = np.array(yaws, dtype=np.float32)
        velocities = np.array(velocities, dtype=np.float32)
        
        features = np.zeros(10, dtype=np.float32)
        
        # 0: x position (average)
        features[0] = np.mean(xs)
        
        # 1: y position (average)
        features[1] = np.mean(ys)
        
        # 2: yaw angle (average)
        features[2] = np.mean(yaws)
        
        # 3: linear velocity (average)
        features[3] = np.mean(velocities)
        
        # 4: angular velocity (estimated from yaw changes)
        if n > 1:
            yaw_diffs = np.diff(yaws)
            yaw_diffs = (yaw_diffs + np.pi) % (2 * np.pi) - np.pi
            features[4] = np.mean(np.abs(yaw_diffs)) / 0.1
        else:
            features[4] = 0.0
        
        # 5: terrain slope (simplified - from path)
        if n > 1:
            path_length = np.sum(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2))
            features[5] = 0.0  # Assume flat for now
        else:
            features[5] = 0.0
        
        # 6: terrain roughness – average curvature plus segment-length coefficient
        # of variation (cv = std/mean).  Unequal segment lengths indicate irregular
        # terrain even on nominally straight paths.
        curvature = self.calculate_roughness(xs, ys)
        segment_lengths = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
        if len(segment_lengths) > 1:
            seg_cv = float(
                np.std(segment_lengths) / (np.mean(segment_lengths) + 1e-6)
            )
        else:
            seg_cv = 0.0
        features[6] = min(curvature + seg_cv * 0.5, 2.0)

        # 7: obstacle density – detour ratio + curvature + direction-change frequency
        # These three signals are non-zero even for straight paths with varying
        # segment lengths or slight direction changes, preventing the "all zeros"
        # failure mode seen when paths are perfectly straight.
        straight_dist = math.sqrt(
            (xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2
        )
        path_length = float(np.sum(segment_lengths))
        detour_ratio = (path_length / max(straight_dist, 0.01)) - 1.0
        # 0.1 rad (~5.7°) is the minimum yaw change treated as a meaningful
        # direction change; smaller values are likely noise from path discretisation.
        direction_changes = float(
            np.sum(np.abs(np.diff(yaws)) > 0.1)
        )
        obs_density = min(
            1.0,
            detour_ratio * 0.5
            + curvature * 0.3
            + (direction_changes / max(n - 2, 1)) * 0.2,
        )
        features[7] = obs_density

        # 8: clearance – inverse of obstacle density with a guaranteed minimum
        features[8] = max(0.1, 3.0 * (1.0 - obs_density))

        # 9: terrain type (default)
        features[9] = 0.0

        return features
    
    def calculate_roughness(self, xs, ys):
        """Calculate terrain roughness from path"""
        if len(xs) < 3:
            return 0.0
        
        total_curvature = 0.0
        for i in range(1, len(xs) - 1):
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
                total_curvature += abs(math.acos(cos_angle))
        
        return total_curvature / max(len(xs) - 2, 1)
    
    def predict_with_model(self, xs, ys, yaws, velocities):
        """Use the neural network for prediction with proper inverse scaling"""
        if self.model is None:
            return None, None
        
        try:
            # Extract features
            features = self.extract_features(xs, ys, yaws, velocities)
            
            # Diagnostic logging
            if self.diagnostic_mode and self.request_count % 5 == 1:
                self.get_logger().info(
                    f"🔍 Features: "
                    f"x={features[0]:.2f}, y={features[1]:.2f}, "
                    f"yaw={features[2]:.2f}, vel={features[3]:.2f}, "
                    f"omega={features[4]:.3f}, slope={features[5]:.3f}, "
                    f"rough={features[6]:.3f}, "
                    f"obs_dens={features[7]:.3f}, "
                    f"clearance={features[8]:.3f}, "
                    f"terrain={features[9]:.1f}"
                )
            
            # Apply input scaling if available
            if self.scaler is not None and isinstance(self.scaler, dict):
                if 'mean' in self.scaler and 'std' in self.scaler:
                    mean = np.array(self.scaler['mean'], dtype=np.float32)
                    std = np.array(self.scaler['std'], dtype=np.float32)
                    # Avoid division by zero
                    std = np.where(std < 1e-6, 1.0, std)
                    features = (features - mean) / std
            
            # Convert to tensor
            input_tensor = torch.FloatTensor(features).unsqueeze(0).to(self.device)
            
            # Run inference without physics constraints so we get raw normalized
            # outputs and can inverse-scale them correctly before applying constraints
            with torch.no_grad():
                if hasattr(self.model, 'forward') and \
                        'apply_physics_constraints' in \
                        self.model.forward.__code__.co_varnames:
                    output = self.model(input_tensor,
                                        apply_physics_constraints=False)
                else:
                    output = self.model(input_tensor)

            # Get raw predictions
            if isinstance(output, torch.Tensor):
                output = output.cpu().numpy().flatten()

                if len(output) >= 2:
                    # Raw normalized network output
                    energy_norm = float(output[0])
                    stability_norm = float(output[1])

                    if self.scaler is not None and isinstance(self.scaler, dict):
                        if 'Y_mean' in self.scaler and 'Y_std' in self.scaler:
                            Y_mean = np.array(self.scaler['Y_mean'],
                                              dtype=np.float32)
                            Y_std = np.array(self.scaler['Y_std'],
                                             dtype=np.float32)

                            # Inverse-scale both outputs first
                            energy = energy_norm * Y_std[0] + Y_mean[0]
                            stability_raw = \
                                stability_norm * Y_std[1] + Y_mean[1]

                            # Then apply physics constraints on the real-scale
                            # values: energy >= 0, stability in [0.1, 1.0]
                            energy = max(0.0, energy)
                            stability = max(0.1, min(1.0, stability_raw))

                            if self.diagnostic_mode and \
                                    self.request_count % 5 == 1:
                                self.get_logger().info(
                                    f"  Raw: {energy_norm:.3f},"
                                    f" {stability_norm:.3f} | "
                                    f"Scaled: {energy:.3f},"
                                    f" {stability:.3f}"
                                )
                        else:
                            energy = max(0.0, energy_norm)
                            stability = max(0.1, min(1.0, stability_norm))
                    else:
                        energy = max(0.0, energy_norm)
                        stability = max(0.1, min(1.0, stability_norm))
                    
                    # Apply energy scaling
                    energy = energy * self.energy_scale
                    
                    # Clip extreme values
                    if energy > 1000:
                        energy = 1000.0
                    if energy < 0:
                        energy = 0.0
                    
                    return energy, stability
            
            return None, None
            
        except Exception as e:
            self.get_logger().error(f"Model inference error: {e}")
            import traceback
            traceback.print_exc()
            return None, None
    
    def predict_physics_based(self, xs, ys, yaws, velocities):
        """Fallback physics-based model"""
        n = len(xs)
        if n < 2:
            return 0.0, 1.0
        
        # Calculate path length
        xs_array = np.array(xs, dtype=np.float32)
        ys_array = np.array(ys, dtype=np.float32)
        dx = np.diff(xs_array)
        dy = np.diff(ys_array)
        path_length = np.sum(np.sqrt(dx**2 + dy**2))
        
        # Average velocity
        avg_velocity = np.mean(velocities) if velocities else 0.5
        
        # Calculate curvature
        total_curvature = 0.0
        if n >= 3:
            for i in range(1, n - 1):
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
                    total_curvature += math.acos(cos_angle)
        
        avg_curvature = total_curvature / max(n - 2, 1)
        
        # Physics-based model with better stability calculation
        robot_mass = 50.0
        wheel_friction = 0.1
        gravity = 9.81
        
        kinetic_energy = 0.5 * robot_mass * avg_velocity**2
        friction_energy = robot_mass * gravity * wheel_friction * path_length
        turning_energy = 0.2 * robot_mass * avg_curvature * path_length
        
        total_energy = kinetic_energy + friction_energy + turning_energy
        
        # FIXED: Better stability calculation
        base_stability = 0.9  # Higher base for clear paths

        # Penalties
        curvature_penalty = 0.4 * min(avg_curvature, 1.0)
        speed_penalty = 0.1 * min(avg_velocity, 2.0)

        stability = base_stability - curvature_penalty - speed_penalty
        stability = max(0.1, min(1.0, stability))
        
        # Add some randomness for challenging paths
        if avg_curvature > 0.5:
            stability = max(0.2, stability - 0.2)
        
        return float(total_energy), float(stability)
    
    def evaluate_callback(self, request, response):
        """Service callback with actual neural network inference"""
        request_start = time.time()
        
        with self.request_lock:
            self.request_count += 1
            req_num = self.request_count
        
        try:
            # Convert inputs
            xs = request.xs
            ys = request.ys
            yaws = request.yaws if request.yaws else []
            velocities = request.velocities if request.velocities else []
            
            # Diagnostic: Log input stats
            if self.diagnostic_mode and req_num % 5 == 1:
                self.get_logger().info(f"🔍 Request #{req_num} - Input stats:")
                self.get_logger().info(f"  xs range: [{min(xs):.2f}, {max(xs):.2f}], len={len(xs)}")
                self.get_logger().info(f"  ys range: [{min(ys):.2f}, {max(ys):.2f}], len={len(ys)}")
            
            # Validate
            if len(xs) < 2 or len(ys) < 2:
                response.energy = 0.0
                response.stability = 1.0
                return response
            
            # Try neural network prediction first
            energy, stability = self.predict_with_model(xs, ys, yaws, velocities)
            inference_type = "neural network"
            
            # Fallback to physics model if neural network fails
            if energy is None or stability is None:
                energy, stability = self.predict_physics_based(xs, ys, yaws, velocities)
                inference_type = "physics"
            
            # Set response
            response.energy = float(energy)
            response.stability = float(stability)
            
            # Update statistics
            with self.request_lock:
                self.success_count += 1
                response_time = time.time() - request_start
                self.response_times.append(response_time)
                if len(self.response_times) > 100:
                    self.response_times.pop(0)
                self.avg_response_time = np.mean(self.response_times)
            
            # Log periodically
            if req_num % 5 == 0 or self.debug_mode:
                self.get_logger().info(
                    f"PINN Service: Request #{req_num}, "
                    f"Energy={response.energy:.2f}J, Stability={response.stability:.3f}, "
                    f"Type={inference_type}, "
                    f"Time={response_time:.3f}s, "
                    f"Avg={self.avg_response_time:.3f}s"
                )
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")
            response.energy = 200.0  # Higher energy for safety
            response.stability = 0.3  # Lower stability for safety
            return response

def main(args=None):
    rclpy.init(args=args)
    
    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    node = PINNService()
    
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.get_logger().info(f"Total requests: {node.request_count}")
        node.get_logger().info(f"Avg response time: {node.avg_response_time:.3f}s")
    finally:
        node.thread_pool.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
