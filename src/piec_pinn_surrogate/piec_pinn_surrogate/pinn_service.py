#!/usr/bin/env python3
"""
PINN Service (full + fixed):
- Keeps your original robust model loader (ament_index + many fallback paths + strict/nostrict + warmup)
- Subscribes to /scan (BEST_EFFORT QoS) and computes obstacle_density/clearance/terrain_type
  conditioned on the requested trajectory (path-conditioned)
- Subscribes to /odom for scan->world transform
- Subscribes to /imu for slope (pitch) and roughness (accel variance over last ~1s)

IMPORTANT:
- Model output[0] is treated as avg_power_W (W). For backward compatibility we return it in response.energy.
  If you want energy (J) for duration T: E ≈ avg_power_W * T.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy

import numpy as np
import torch
import torch.nn as nn
import os
import math
import time
from concurrent.futures import ThreadPoolExecutor
import threading
import sys
from collections import deque

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import model
try:
    from pinn_model import PhysicsInformedPINN
    PINN_MODEL_AVAILABLE = True
except ImportError as e:
    PINN_MODEL_AVAILABLE = False
    print(f"⚠️ Could not import PhysicsInformedPINN: {e}")

try:
    from piec_pinn_surrogate_msgs.srv import EvaluateTrajectory
except ImportError:
    # dev fallback
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


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def quat_to_pitch(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


def point_to_segment_distance(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


class PINNService(Node):
    def __init__(self):
        super().__init__('pinn_service')

        service_name = '/evaluate_trajectory'

        # -------- Parameters (keep like your original) --------
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('stability_scale', 1.0)
        self.declare_parameter('energy_scale', 1.0)
        self.declare_parameter('diagnostic_mode', True)

        # topics (remappable)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/openzen/data')
        self.declare_parameter('odom_topic', '/odometry')

        # obstacle/path params
        self.declare_parameter('path_band_m', 0.6)
        self.declare_parameter('density_range_m', 3.0)
        self.declare_parameter('scan_max_range_m', 8.0)

        # roughness params
        self.declare_parameter('roughness_window_s', 1.0)
        self.declare_parameter('roughness_sigma_ref', 1.5)
        self.declare_parameter('roughness_use_3d_norm', True)

        self.debug_mode = bool(self.get_parameter('debug_mode').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.use_gpu = bool(self.get_parameter('use_gpu').value)
        self.stability_scale = float(self.get_parameter('stability_scale').value)
        self.energy_scale = float(self.get_parameter('energy_scale').value)
        self.diagnostic_mode = bool(self.get_parameter('diagnostic_mode').value)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.path_band_m = float(self.get_parameter('path_band_m').value)
        self.density_range_m = float(self.get_parameter('density_range_m').value)
        self.scan_max_range_m = float(self.get_parameter('scan_max_range_m').value)

        self.roughness_window_s = float(self.get_parameter('roughness_window_s').value)
        self.roughness_sigma_ref = float(self.get_parameter('roughness_sigma_ref').value)
        self.roughness_use_3d_norm = bool(self.get_parameter('roughness_use_3d_norm').value)

        # -------- Thread pool / service --------
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        self.request_lock = threading.Lock()

        self.srv = self.create_service(
            EvaluateTrajectory,
            service_name,
            self.evaluate_callback,
            qos_profile=rclpy.qos.qos_profile_services_default
        )

        # -------- Model --------
        self.model = None
        self.scaler = None
        self.device = torch.device('cuda' if torch.cuda.is_available() and self.use_gpu else 'cpu')

        self.get_logger().info(f"Using device: {self.device}")
        self.get_logger().info(f"CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            self.get_logger().info(f"GPU: {torch.cuda.get_device_name(0)}")

        self.load_model()

        # -------- Live state --------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_ready = False

        self.slope = 0.0
        self.roughness = 0.0
        self.imu_ready = False
        self.accel_hist = deque()

        self.last_scan_msg = None
        self.scan_ready = False

        # -------- Subscriptions (QoS FIX for scan) --------
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,   # FIX: avoid RELIABILITY mismatch
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, scan_qos)

        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 50)

        # -------- Statistics --------
        self.request_count = 0
        self.avg_response_time = 0.0
        self.response_times = []

        self.get_logger().info(f"✅ PINN Service READY at: {service_name}")
        self.get_logger().info(f"Topics: scan={self.scan_topic}, imu={self.imu_topic}, odom={self.odom_topic}")
        self.get_logger().info(
            f"Roughness: window={self.roughness_window_s:.2f}s sigma_ref={self.roughness_sigma_ref:.2f} use3d={self.roughness_use_3d_norm}"
        )

    # ----------------- Robust model loader (like your original) -----------------
    def create_compatible_model(self, input_dim, hidden_dim):
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

    def warmup_model(self, input_dim):
        if self.model is None:
            return
        try:
            dummy = torch.randn(1, input_dim).to(self.device)
            with torch.no_grad():
                for _ in range(3):
                    _ = self.model(dummy)  # may be constrained forward; OK
            if self.device.type == 'cuda':
                torch.cuda.synchronize()
        except Exception as e:
            self.get_logger().debug(f"Warmup failed (non-critical): {e}")

    def load_model(self):
        """Load checkpoint from many possible paths (ament_index + fallback paths)."""
        try:
            model_paths = []
            if self.model_path:
                model_paths.append(self.model_path)

            # ament_index path (installed)
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('piec_pinn_surrogate')
                model_paths.append(os.path.join(pkg_share, 'models', 'pinn_physics.pt'))
            except Exception:
                pass

            # fallback paths (keep your originals)
            model_paths += [
                os.path.join(os.path.dirname(__file__), '..', 'models', 'pinn_physics.pt'),
                os.path.join(os.path.dirname(__file__), 'models', 'pinn_physics.pt'),
                '/home/amjad/PIEC_2d/src/piec_pinn_surrogate/models/pinn_physics.pt',
                '/home/amjad/PIEC_2d/install/piec_pinn_surrogate/share/piec_pinn_surrogate/models/pinn_physics.pt',
                'pinn_physics.pt'
            ]

            for p in model_paths:
                if os.path.exists(p):
                    self.get_logger().info(f"Loading PINN model from: {p}")

                    checkpoint = torch.load(p, map_location=self.device)
                    input_dim = int(checkpoint.get('input_dim', 11))
                    hidden_dim = int(checkpoint.get('hidden_dim', 128))
                    self.get_logger().info(f"Model architecture: input_dim={input_dim}, hidden_dim={hidden_dim}")

                    if PINN_MODEL_AVAILABLE:
                        self.model = PhysicsInformedPINN(input_dim=input_dim, hidden_dim=hidden_dim).to(self.device)
                        self.get_logger().info("Created PhysicsInformedPINN instance")
                    else:
                        self.model = self.create_compatible_model(input_dim, hidden_dim).to(self.device)
                        self.get_logger().info("Created compatible fallback model")

                    state_dict = checkpoint['model_state'] if 'model_state' in checkpoint else checkpoint

                    try:
                        self.model.load_state_dict(state_dict, strict=True)
                        self.get_logger().info("✅ Model loaded with strict=True")
                    except Exception as e:
                        self.get_logger().warn(f"Strict loading failed: {e}")
                        self.get_logger().info("Attempting strict=False...")
                        self.model.load_state_dict(state_dict, strict=False)
                        self.get_logger().info("✅ Model loaded with strict=False")

                    self.scaler = checkpoint.get('scaler', None)
                    if self.scaler:
                        self.get_logger().info("Scaler loaded successfully")
                        if 'Y_mean' in self.scaler:
                            self.get_logger().info(f"Y_mean: {self.scaler['Y_mean']}")
                            self.get_logger().info(f"Y_std: {self.scaler['Y_std']}")

                    self.model.eval()
                    if 'features' in checkpoint:
                        self.get_logger().info(f"Features: {checkpoint['features']}")
                    if 'targets' in checkpoint:
                        self.get_logger().info(f"Targets: {checkpoint['targets']}")

                    self.warmup_model(input_dim)
                    self.get_logger().info(f"✅ PINN model loaded successfully on {self.device}")
                    return

            self.get_logger().warn("PINN model not found, using fallback outputs only")
        except Exception as e:
            self.get_logger().error(f"Failed to load PINN model: {e}")
            import traceback
            traceback.print_exc()

    # ---------------- sensors ----------------
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_ready = True

    def imu_cb(self, msg: Imu):
        # slope from orientation
        q = msg.orientation
        normq = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if normq > 0.5:
            self.slope = float(quat_to_pitch(q))
            self.imu_ready = True

        # roughness from linear acceleration variance (last ~window)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        if self.roughness_use_3d_norm:
            a_metric = math.sqrt(ax*ax + ay*ay + az*az)
        else:
            a_metric = math.sqrt(ax*ax + ay*ay)

        now_s = self.get_clock().now().nanoseconds * 1e-9
        self.accel_hist.append((now_s, a_metric))
        cutoff = now_s - self.roughness_window_s
        while self.accel_hist and self.accel_hist[0][0] < cutoff:
            self.accel_hist.popleft()

        if len(self.accel_hist) >= 10:
            vals = np.array([v for _, v in self.accel_hist], dtype=np.float32)
            sigma = float(np.std(vals))
            self.roughness = float(max(0.0, min(1.0, sigma / max(1e-6, self.roughness_sigma_ref))))
        else:
            self.roughness = 0.0

    def scan_cb(self, msg: LaserScan):
        self.last_scan_msg = msg
        self.scan_ready = True

    # ---------------- scan->world & path-conditioned obstacles ----------------
    def scan_points_world(self):
        if (not self.scan_ready) or (self.last_scan_msg is None) or (not self.odom_ready):
            return None

        msg = self.last_scan_msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        rmax = min(float(msg.range_max), float(self.scan_max_range_m))

        valid = (ranges > float(msg.range_min)) & (ranges < rmax)
        idx = np.where(valid)[0]
        if idx.size == 0:
            return None

        angles = float(msg.angle_min) + idx.astype(np.float32) * float(msg.angle_increment)
        r = ranges[idx]

        lx = r * np.cos(angles)
        ly = r * np.sin(angles)

        cy = math.cos(self.robot_yaw)
        sy = math.sin(self.robot_yaw)

        wx = self.robot_x + (lx * cy - ly * sy)
        wy = self.robot_y + (lx * sy + ly * cy)

        return np.stack([wx, wy], axis=1)

    def obstacle_features_for_path(self, xs, ys):
        pts = self.scan_points_world()
        if pts is None or len(xs) < 2:
            return 0.0, 3.0, 0.0

        dx = pts[:, 0] - self.robot_x
        dy = pts[:, 1] - self.robot_y
        dist_robot = np.sqrt(dx * dx + dy * dy)
        use = dist_robot < self.density_range_m
        pts = pts[use]
        if pts.shape[0] == 0:
            return 0.0, 3.0, 0.0

        xs = list(xs)
        ys = list(ys)
        segs = list(zip(xs[:-1], ys[:-1], xs[1:], ys[1:]))

        dmins = []
        for px, py in pts:
            dmin = 1e9
            for ax, ay, bx, by in segs:
                d = point_to_segment_distance(px, py, ax, ay, bx, by)
                if d < dmin:
                    dmin = d
            dmins.append(dmin)

        dmins = np.array(dmins, dtype=np.float32)

        clearance = float(np.min(dmins))
        clearance = float(max(0.05, min(10.0, clearance)))

        obstacle_density = float(np.mean(dmins < self.path_band_m))
        obstacle_density = float(max(0.0, min(1.0, obstacle_density)))

        std = float(np.std(dist_robot[use])) if use.any() else 0.0
        if std < 0.2:
            terrain_type = 0.0
        elif std < 0.5:
            terrain_type = 1.0
        else:
            terrain_type = 2.0

        return obstacle_density, clearance, terrain_type

    # ---------------- inference features ----------------
    def extract_features(self, xs, ys, yaws, velocities):
        n = len(xs)
        if n < 2:
            return np.zeros(11, dtype=np.float32)

        xs_np = np.array(xs, dtype=np.float32)
        ys_np = np.array(ys, dtype=np.float32)

        if yaws and len(yaws) >= n:
            yaws_np = np.array(yaws, dtype=np.float32)
        else:
            yaws_np = np.zeros(n, dtype=np.float32)
            for i in range(1, n):
                yaws_np[i] = math.atan2(ys_np[i] - ys_np[i - 1], xs_np[i] - xs_np[i - 1])
            yaws_np[0] = yaws_np[1] if n > 1 else 0.0

        if velocities and len(velocities) >= n:
            v_np = np.array(velocities, dtype=np.float32)
        else:
            v_np = np.full(n, 0.3, dtype=np.float32)

        seg = np.sqrt(np.diff(xs_np) ** 2 + np.diff(ys_np) ** 2)
        path_length = float(np.sum(seg))

        dyaw = np.diff(yaws_np)
        dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi
        omega_sum = float(np.sum(np.abs(dyaw))) if n > 1 else 0.0

        obs_dens, clear, terrain = self.obstacle_features_for_path(xs, ys)

        feat = np.zeros(11, dtype=np.float32)
        feat[0] = float(np.mean(xs_np))
        feat[1] = float(np.mean(ys_np))
        feat[2] = float(np.mean(yaws_np))
        feat[3] = float(np.mean(v_np))
        feat[4] = omega_sum
        feat[5] = float(self.slope) if self.imu_ready else 0.0
        feat[6] = float(self.roughness)  # live 0..1
        feat[7] = float(obs_dens)
        feat[8] = float(clear)
        feat[9] = float(terrain)
        feat[10] = float(path_length)
        return feat

    def predict(self, features):
        if self.model is None:
            return 0.0, 1.0

        x_raw = np.array(features, dtype=np.float32)

        # Normalize inputs (X)
        if self.scaler and isinstance(self.scaler, dict) and 'mean' in self.scaler and 'std' in self.scaler:
            meanX = np.array(self.scaler['mean'], dtype=np.float32)
            stdX = np.array(self.scaler['std'], dtype=np.float32)
            stdX = np.where(stdX < 1e-6, 1.0, stdX)
            x_norm = (x_raw - meanX) / stdX
        else:
            x_norm = x_raw

        x_t = torch.from_numpy(x_norm).float().unsqueeze(0).to(self.device)

        with torch.no_grad():
            # IMPORTANT: get RAW outputs in normalized Y-space
            try:
                y_norm = self.model(x_t, apply_physics_constraints=False)
            except TypeError:
                # if CompatiblePINN fallback
                y_norm = self.model(x_t)

            y_norm = y_norm.cpu().numpy().flatten()

        # Inverse-scale outputs (Y)
        if self.scaler and isinstance(self.scaler, dict) and 'Y_mean' in self.scaler and 'Y_std' in self.scaler:
            meanY = np.array(self.scaler['Y_mean'], dtype=np.float32)
            stdY = np.array(self.scaler['Y_std'], dtype=np.float32)
            stdY = np.where(stdY < 1e-6, 1.0, stdY)

            avg_power_W = float(y_norm[0] * stdY[0] + meanY[0])
            stability = float(y_norm[1] * stdY[1] + meanY[1])
        else:
            avg_power_W = float(y_norm[0])
            stability = float(y_norm[1])

        # Apply scales
        avg_power_W *= self.energy_scale
        stability *= self.stability_scale

        # Clamp
        avg_power_W = max(0.0, avg_power_W)
        stability = max(0.1, min(1.0, stability))

        return avg_power_W, stability

    # ---------------- service callback ----------------
    def evaluate_callback(self, request, response):
        request_start = time.time()
        with self.request_lock:
            self.request_count += 1
            req_num = self.request_count

        xs = request.xs
        ys = request.ys
        yaws = request.yaws if getattr(request, 'yaws', None) else []
        vels = request.velocities if getattr(request, 'velocities', None) else []

        if len(xs) < 2 or len(ys) < 2:
            response.energy = 0.0
            response.stability = 1.0
            return response

        feats = self.extract_features(xs, ys, yaws, vels)
        avg_power_W, stability = self.predict(feats)

        response.energy = float(avg_power_W)
        response.stability = float(stability)

        dt = time.time() - request_start
        with self.request_lock:
            self.response_times.append(dt)
            if len(self.response_times) > 100:
                self.response_times.pop(0)
            self.avg_response_time = float(np.mean(self.response_times))

        if self.diagnostic_mode and req_num % 5 == 0:
            self.get_logger().info(
                f"Req#{req_num}: power={response.energy:.1f}W stability={response.stability:.3f} "
                f"obs={feats[7]:.2f} clear={feats[8]:.2f} slope={feats[5]:.3f} rough={feats[6]:.2f} "
                f"scan={'OK' if self.scan_ready else 'NO'} odom={'OK' if self.odom_ready else 'NO'} imu={'OK' if self.imu_ready else 'NO'} "
                f"time={dt:.3f}s"
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = PINNService()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.thread_pool.shutdown(wait=True)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
