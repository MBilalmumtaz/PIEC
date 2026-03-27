#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time
from numpy.linalg import LinAlgError

# Use numpy.linalg.cholesky (always returns lower-triangular) so that no
# SciPy version constraint is imposed.  If SciPy is available we still
# prefer its Cholesky because it raises LinAlgError on failure (matching
# the existing exception handler), whereas numpy raises numpy.linalg.LinAlgError
# which is the same class – both are subclasses of numpy.linalg.LinAlgError.
try:
    from scipy.linalg import cholesky as _scipy_cholesky

    def cholesky(M):
        """Thin wrapper: always return lower-triangular factor."""
        return _scipy_cholesky(M, lower=True)
except ImportError:
    def cholesky(M):           # noqa: F811
        """numpy fallback – always lower-triangular."""
        return np.linalg.cholesky(M)

class UKFNode(Node):
    def __init__(self):
        super().__init__('piec_ukf_node')
        # Topics (configurable)
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('publish_topic', '/ukf/odom')
        self.declare_parameter('dt', 0.05)
        
        # CRITICAL: New parameters for real robot
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('imu_yaw_offset', 0.0)
        self.declare_parameter('invert_yaw', False)   # set True to flip yaw 180° for inverted IMU mount
        self.declare_parameter('use_wheel_velocity', True)
        self.declare_parameter('debug_mode', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pub_topic = self.get_parameter('publish_topic').value
        self.dt = float(self.get_parameter('dt').value)
        self.use_imu_orientation = self.get_parameter('use_imu_orientation').value
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.imu_yaw_offset = float(self.get_parameter('imu_yaw_offset').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.use_wheel_velocity = self.get_parameter('use_wheel_velocity').value
        self.debug_mode = self.get_parameter('debug_mode').value

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.pub = self.create_publisher(Odometry, self.pub_topic, 10)
        # Health topic: [trace, is_healthy (1.0/0.0), max_eigenvalue, mahalanobis_last]
        self.health_pub = self.create_publisher(Float64MultiArray, '/ukf/health', 10)

        # state: x, y, yaw, v, w (angular velocity)
        # CHANGED: Now 5 dimensions
        self.x = np.zeros(5)
        
        # Initialize with orientation from IMU if available
        self.x[2] = self.initial_yaw  # yaw
        
        # Covariance matrix - MUST BE 5x5
        self.P = np.diag([0.2, 0.2, 0.05, 0.1, 0.01])  # 5x5
        
        # Process noise covariance - INCREASED for stability
        self.Q = np.diag([
            0.1,    # x process noise (increased from 1e-4)
            0.1,    # y process noise (increased from 1e-4)
            0.05,   # yaw process noise (increased from 1e-5)
            0.2,    # v process noise (increased from 5e-3)
            0.2     # w process noise (increased from 1e-4)
        ])
        
        # Measurement noise
        self.R_pose = np.diag([0.01, 0.01, 0.005])  # For position measurements (x,y,yaw)
        self.R_yawrate = np.array([[0.01]])  # For angular velocity measurements
        
        # UKF params
        self.alpha = 1e-3; self.kappa = 0; self.beta = 2
        
        # Initialization flags
        self.initialized = False
        self.imu_initialized = False
        self.initial_imu_yaw = 0.0
        self.last_odom_time = None
        self.initialization_count = 0
        
        # Current IMU data storage
        self.current_imu_yaw = 0.0
        self.current_imu_wz = 0.0
        
        # Velocity tracking
        self.last_velocity = 0.0
        self.last_yaw = 0.0
        
        # Measurement rejection tracking
        self.rejection_count = 0
        self.measurement_count = 0
        self.last_stats_print = self.get_clock().now()
        
        # Debug message throttling
        self._imu_debug_counter = 0

        # Rate-limit predict step: wall-clock timestamp of last predict call.
        # Without this, a predict+update is run for every incoming measurement
        # (50 Hz IMU + 10 Hz odom × 2 = ~70/s), accumulating process noise far
        # faster than intended and causing covariance blow-ups.
        self._last_predict_wall_time: float = 0.0

        # Health tracking: last Mahalanobis distance (used in health topic)
        self._last_mahalanobis: float = 0.0
        # UKF is healthy when trace < 10 and no NaN/Inf
        self._ukf_healthy: bool = False
        
        self.get_logger().info(f'UKF node started. odom_topic: {self.odom_topic} imu_topic: {self.imu_topic}')
        self.get_logger().info(f'State dimension: {self.x.size}, Use IMU orientation: {self.use_imu_orientation}, '
                               f'invert_yaw: {self.invert_yaw}')

    def quat_to_yaw(self, q):
        """Fixed quaternion to yaw conversion"""
        # Handle quaternion normalization issues
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 0.001:
            return 0.0
        
        x = q.x / norm
        y = q.y / norm
        z = q.z / norm
        w = q.w / norm
        
        # Use atan2 for full range [-π, π]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def odom_cb(self, msg: Odometry):
        """Handle odometry callback"""
        if not self.initialized and self.imu_initialized:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            
            # Use IMU yaw if available, otherwise use odometry
            if self.imu_initialized:
                yaw = self.current_imu_yaw
            else:
                yaw = self.quat_to_yaw(msg.pose.pose.orientation)
            
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            
            self.x[0] = px
            self.x[1] = py
            self.x[2] = yaw
            self.x[3] = v if abs(v) > 0.01 else 0.0
            self.x[4] = w
            
            self.initialized = True
            self.last_velocity = v
            self.last_yaw = yaw
            
            if self.debug_mode:
                self.get_logger().info(f'✅ UKF initialized: x={px:.2f}, y={py:.2f}, yaw={yaw:.2f}, v={v:.2f}, w={w:.2f}')
        
        # For measurement update, use position from odometry and orientation from IMU
        if self.initialized:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            
            # Use IMU yaw if enabled, otherwise use odometry yaw
            if self.use_imu_orientation and self.imu_initialized:
                yaw = self.current_imu_yaw
            else:
                yaw = self.quat_to_yaw(msg.pose.pose.orientation)
            
            z = np.array([px, py, yaw])
            
            # Update velocity from odometry
            self.x[3] = msg.twist.twist.linear.x
            self.x[4] = msg.twist.twist.angular.z
            
            self.ukf_update(z, self.R_pose, meas_type='pose')
            
            # Also update with angular velocity from IMU if available
            if self.imu_initialized:
                z_w = np.array([self.current_imu_wz])
                # Validate before updating
                z_pred = np.array([self.x[4]])  # Predicted angular velocity
                if self.validate_measurement(z_w, z_pred, self.R_yawrate):
                    self.ukf_update(z_w, self.R_yawrate, meas_type='angular_velocity')

    def imu_cb(self, msg: Imu):
        """IMU callback with enhanced diagnostics for real robot"""
        # Calculate yaw from quaternion
        yaw = self.quat_to_yaw(msg.orientation)
        
        if not self.imu_initialized:
            self.initial_imu_yaw = yaw
            self.imu_initialized = True
            if self.debug_mode:
                self.get_logger().info(f"IMU initialized with yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
                # Log raw quaternion for debugging the 54° issue
                self.get_logger().info(
                    f"IMU raw quaternion: w={msg.orientation.w:.4f}, "
                    f"x={msg.orientation.x:.4f}, y={msg.orientation.y:.4f}, z={msg.orientation.z:.4f}"
                )
                # Log linear acceleration to verify gravity vector
                self.get_logger().info(
                    f"IMU gravity check: ax={msg.linear_acceleration.x:.2f}, "
                    f"ay={msg.linear_acceleration.y:.2f}, az={msg.linear_acceleration.z:.2f} m/s²"
                )
        
        # Store current IMU values
        # Remove startup bias, then apply static frame correction
        self.current_imu_yaw = yaw - self.initial_imu_yaw + self.imu_yaw_offset

        # Optional 180° flip for IMUs mounted upside-down or with inverted z-axis
        if self.invert_yaw:
            self.current_imu_yaw = self.current_imu_yaw + math.pi
        
        # Normalize yaw to [-π, π] to prevent angle wrapping issues
        self.current_imu_yaw = math.atan2(math.sin(self.current_imu_yaw), math.cos(self.current_imu_yaw))
        
        self.current_imu_wz = msg.angular_velocity.z
        
        # Debug: Log yaw transformation (every 100th message to avoid spam)
        if self.debug_mode:
            self._imu_debug_counter += 1
            if self._imu_debug_counter % 100 == 0:
                self.get_logger().info(
                    f"IMU yaw transform: raw={yaw:.3f} → initial={self.initial_imu_yaw:.3f} "
                    f"→ offset={self.imu_yaw_offset:.3f} → final={self.current_imu_yaw:.3f} rad"
                )
        
        # If initialized, update UKF with IMU angular velocity
        if self.initialized:
            z = np.array([self.current_imu_wz])
            z_pred = np.array([self.x[4]])
            if self.validate_measurement(z, z_pred, self.R_yawrate):
                self.ukf_update(z, self.R_yawrate, meas_type='angular_velocity')

    def motion_model(self, x):
        """Motion model - now with 5 states"""
        dt = self.dt
        xp = np.copy(x)
        
        # Extract states
        theta = x[2]  # yaw
        v = x[3]      # linear velocity
        w = x[4]      # angular velocity
        
        # Update position
        if abs(w) < 0.001:  # Straight line motion
            xp[0] = x[0] + v * math.cos(theta) * dt
            xp[1] = x[1] + v * math.sin(theta) * dt
        else:  # Curved motion
            radius = v / w if abs(w) > 0.001 else float('inf')
            xp[0] = x[0] + radius * (math.sin(theta + w * dt) - math.sin(theta))
            xp[1] = x[1] + radius * (math.cos(theta) - math.cos(theta + w * dt))
        
        # Update orientation
        xp[2] = theta + w * dt
        xp[2] = math.atan2(math.sin(xp[2]), math.cos(xp[2]))  # Normalize
        
        # Velocity remains unchanged (process noise will update)
        
        return xp

    # Minimum eigenvalue floor: keeps P positive-definite after clamping.
    _MIN_EIGENVALUE = 1e-6

    # Maximum allowed variance per state dimension.  Values beyond these indicate
    # numerical runaway and the covariance is clamped rather than reset outright.
    _P_MAX_DIAG = np.array([
        25.0,  # x position variance ceiling (m²)
        25.0,  # y position variance ceiling (m²)
         2.5,  # yaw variance ceiling (rad²) – tight because yaw errors cause heading bugs
        10.0,  # linear velocity variance ceiling ((m/s)²)
        10.0,  # angular velocity variance ceiling ((rad/s)²)
    ])

    def _clamp_covariance(self):
        """Clamp P eigenvalues to [_MIN_EIGENVALUE, P_MAX] to keep it bounded and PD.

        Called after every ukf_update to prevent numerical blow-ups without
        triggering a full UKF reset for mild divergence.
        """
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(self.P)
            eigenvalues = np.maximum(eigenvalues, self._MIN_EIGENVALUE)
            p_max = float(np.max(self._P_MAX_DIAG))
            eigenvalues = np.minimum(eigenvalues, p_max)
            self.P = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
            self.P = 0.5 * (self.P + self.P.T)
        except (LinAlgError, ValueError):
            # Fallback: clamp diagonal entries only
            p_max = float(np.max(self._P_MAX_DIAG))
            for i in range(self.P.shape[0]):
                self.P[i, i] = float(np.clip(self.P[i, i], self._MIN_EIGENVALUE, p_max))

    def check_and_reset_covariance(self):
        """Check covariance health and reset if needed"""
        # Check for NaN or Inf
        if np.any(np.isnan(self.P)) or np.any(np.isinf(self.P)):
            self.get_logger().error("🚨 UKF covariance has NaN/Inf - RESETTING")
            self.reset_ukf()
            return True
        
        # Check trace (total uncertainty).  Use a generous threshold so the
        # clamping logic above prevents runaway before a full reset is needed.
        # The old threshold of 10.0 was causing resets at normal operating
        # covariance levels (trace grows to ~7 without obstacles from 70×/s
        # predict accumulation).  With rate-limited predict the steady-state
        # trace is ~0.5; reset only on clear divergence.
        trace = np.trace(self.P)
        if trace > 100.0:
            self.get_logger().warn(f"⚠️ UKF uncertainty too high (trace={trace:.2f}) - RESETTING")
            self.reset_ukf()
            return True
        
        # Check condition number
        try:
            eigenvalues = np.linalg.eigvals(self.P)
            eigenvalues = eigenvalues[eigenvalues > 1e-12]  # Filter out numerical zeros
            if len(eigenvalues) > 0:
                condition_number = np.max(eigenvalues) / np.min(eigenvalues)
                if condition_number > 1e8:
                    self.get_logger().warn(f"⚠️ UKF ill-conditioned (κ={condition_number:.2e}) - RESETTING")
                    self.reset_ukf()
                    return True
        except (LinAlgError, ValueError) as e:
            self.get_logger().warn(f"⚠️ Eigenvalue calculation failed: {e} - skipping condition number check")
        
        return False

    def reset_ukf(self):
        """Soft-reset UKF: re-seed covariance without altering current yaw.

        Only velocities are zeroed so the controller does not receive a sudden
        yaw jump.  Position is kept intact because odom drift is small compared
        with a yaw flip.  Covariance is re-seeded to modest initial values so
        the filter recovers quickly from divergence.
        """
        self.get_logger().info("🔄 Soft-resetting UKF covariance (yaw preserved)")

        # Keep x[0..2] (x, y, yaw) unchanged to avoid heading jumps.
        self.x[3] = 0.0  # v = 0
        self.x[4] = 0.0  # w = 0

        # Re-seed covariance to modest initial values
        self.P = np.diag([
            0.05,   # x variance (m²)
            0.05,   # y variance (m²)
            0.05,   # yaw variance (rad²) – modest so heading is trusted
            0.1,    # v variance ((m/s)²)
            0.05,   # w variance ((rad/s)²)
        ])

    def validate_measurement(self, z, z_pred, R):
        """Check if measurement is valid with adaptive threshold"""
        self.measurement_count += 1
        
        innovation = z - z_pred
        
        # Mahalanobis distance
        S = R  # Measurement covariance
        try:
            mahalanobis_dist = float(np.sqrt(innovation.T @ np.linalg.inv(S) @ innovation))
        except (LinAlgError, ValueError) as e:
            self.get_logger().warn(f"⚠️ Mahalanobis calculation failed: {e} - accepting measurement")
            return True  # If calculation fails, accept measurement

        # Store most recent Mahalanobis distance for health topic
        self._last_mahalanobis = mahalanobis_dist

        # Adaptive threshold based on rejection rate
        base_threshold = 5.0  # Changed from 3.0
        rejection_rate = self.rejection_count / max(self.measurement_count, 1)
        
        # If rejection rate > 30%, relax threshold further
        if rejection_rate > 0.3:
            threshold = base_threshold * 1.5  # 7.5
        else:
            threshold = base_threshold
        
        # Check if measurement should be rejected
        is_rejected = mahalanobis_dist > threshold
        
        if is_rejected:
            self.rejection_count += 1
        
        # Print stats every 10 seconds (only in debug mode to reduce log spam)
        current_time = self.get_clock().now()
        if self.debug_mode and (current_time - self.last_stats_print).nanoseconds > 10e9:
            self.get_logger().info(
                f"📊 UKF Stats: {self.rejection_count}/{self.measurement_count} rejected "
                f"({rejection_rate*100:.1f}%), threshold={threshold:.1f}"
            )
            self.last_stats_print = current_time
            # Reset counters after printing
            self.rejection_count = 0
            self.measurement_count = 0
        
        return not is_rejected

    def generate_sigma_points(self, x, P):
        """Generate sigma points with numerical stability"""
        n = len(x)
        lam = self.alpha**2 * (n + self.kappa) - n
        c = n + lam
        
        # Ensure P is symmetric (numerical errors can break symmetry)
        P = 0.5 * (P + P.T)
        
        # Add regularization to ensure positive definiteness
        min_eigenvalue = np.min(np.linalg.eigvals(P))
        if min_eigenvalue < self._MIN_EIGENVALUE:
            self.get_logger().warn(f"⚠️ UKF covariance regularized (min eigenvalue: {min_eigenvalue:.2e})")
            P += (self._MIN_EIGENVALUE - min_eigenvalue + 1e-9) * np.eye(n)
        
        try:
            S = cholesky(P * c)
        except LinAlgError:
            self.get_logger().error("❌ Cholesky failed, using eigendecomposition fallback")
            # Fallback: Use eigenvalue decomposition
            eigenvalues, eigenvectors = np.linalg.eigh(P * c)
            eigenvalues = np.maximum(eigenvalues, self._MIN_EIGENVALUE)
            S = eigenvectors @ np.diag(np.sqrt(eigenvalues))
        
        # Generate sigma points
        X = np.zeros((2 * n + 1, n))
        X[0] = x
        for i in range(n):
            X[i + 1] = x + S[:, i]
            X[i + 1 + n] = x - S[:, i]
        
        # Weights (standard UKF formulation)
        Wm = np.zeros(2 * n + 1)
        Wc = np.zeros(2 * n + 1)
        Wm[0] = lam / c
        Wc[0] = lam / c + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * n + 1):
            Wm[i] = 1.0 / (2 * c)
            Wc[i] = 1.0 / (2 * c)
        
        return X, Wm, Wc

    def ukf_predict(self):
        """UKF predict step - FIXED for 5 dimensions"""
        # Generate sigma points
        X, Wm, Wc = self.generate_sigma_points(self.x, self.P)
        
        # Predict sigma points
        Xp = np.array([self.motion_model(xi) for xi in X])
        
        # Calculate predicted mean
        x_pred = np.zeros_like(self.x)
        for i in range(Xp.shape[0]):
            x_pred += Wm[i] * Xp[i]
        
        # Calculate predicted covariance
        P_pred = np.zeros_like(self.P)
        for i in range(Xp.shape[0]):
            dx = (Xp[i] - x_pred).reshape(-1, 1)
            P_pred += Wc[i] * (dx @ dx.T)
        
        # Add process noise
        P_pred += self.Q
        
        self.x = x_pred
        self.P = P_pred

    def ukf_update(self, z, R, meas_type='pose'):
        """UKF update step with rate-limited predict and post-update clamping.

        Root-cause fix for covariance blow-ups
        ----------------------------------------
        Previously every incoming measurement triggered a full predict+update.
        At 50 Hz (IMU) + 10 Hz × 2 (odom pose + odom angular-vel) = 70 calls/s,
        process noise Q was accumulated 70 times per second instead of the
        intended ~10–20 times.  After 1 s, trace(P) grew to > 70×trace(Q) = 35,
        well above the old reset threshold of 10, causing very frequent resets
        and the observed yaw jumps.

        Fix: only run ukf_predict() if at least self.dt seconds have elapsed
        since the last predict call (wall-clock).  Measurement-only updates
        (without a new predict) are still applied so no sensor data is dropped.
        """
        # Check covariance health BEFORE update
        if self.check_and_reset_covariance():
            return  # Skip this update after reset

        # Rate-limited predict: run at most 1/dt times per second
        now = time.time()
        if now - self._last_predict_wall_time >= self.dt:
            self.ukf_predict()
            self._last_predict_wall_time = now
        
        # Generate sigma points
        X, Wm, Wc = self.generate_sigma_points(self.x, self.P)
        
        # Measurement function
        if meas_type == 'pose':
            # Measurement: [x, y, yaw]
            Z = np.array([[xi[0], xi[1], xi[2]] for xi in X])
            n_z = 3
        elif meas_type == 'angular_velocity':
            # Measurement: [angular_velocity]
            Z = np.array([[xi[4]] for xi in X])
            n_z = 1
        else:
            return
        
        # Calculate predicted measurement mean
        z_pred = np.zeros(n_z)
        for i in range(X.shape[0]):
            z_pred += Wm[i] * Z[i]
        
        # Calculate innovation covariance
        Pzz = np.zeros((n_z, n_z))
        Pxz = np.zeros((self.x.size, n_z))
        
        for i in range(X.shape[0]):
            dz = (Z[i] - z_pred).reshape(-1, 1)
            dx = (X[i] - self.x).reshape(-1, 1)
            
            Pzz += Wc[i] * (dz @ dz.T)
            Pxz += Wc[i] * (dx @ dz.T)
        
        # Add measurement noise
        Pzz += R
        
        # Calculate Kalman gain
        K = Pxz @ np.linalg.inv(Pzz)
        
        # Update state with normalized innovation
        innov = (z - z_pred).reshape(-1, 1)
        # Normalize yaw innovation for pose measurements to prevent wrap-around divergence
        if meas_type == 'pose':
            innov[2, 0] = math.atan2(math.sin(innov[2, 0]), math.cos(innov[2, 0]))
        self.x = self.x + (K @ innov).flatten()
        
        # Update covariance
        self.P = self.P - K @ Pzz @ K.T
        
        # Ensure P remains symmetric (numerical stability)
        self.P = 0.5 * (self.P + self.P.T)

        # Clamp eigenvalues to keep P bounded and positive-definite.
        # This replaces the previous strategy of resetting the entire UKF
        # whenever P blew up, which caused the observed yaw jumps.
        self._clamp_covariance()
        
        # Normalize yaw
        self.x[2] = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
        
        self.publish_ukf()

    def publish_ukf(self):
        """Publish UKF odometry and health."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.orientation = self.yaw_to_quat(self.x[2])
        
        # Velocity
        msg.twist.twist.linear.x = float(self.x[3])
        msg.twist.twist.angular.z = float(self.x[4])
        
        # Covariance (6x6 matrix for pose)
        cov = np.zeros(36)
        # Position covariance
        cov[0] = float(self.P[0,0]); cov[1] = float(self.P[0,1]); cov[6] = float(self.P[1,0]); cov[7] = float(self.P[1,1])
        # Orientation covariance
        cov[35] = float(self.P[2,2])
        msg.pose.covariance = cov.tolist()
        
        self.pub.publish(msg)

        # Publish health: [trace, is_healthy, max_eigenvalue, last_mahalanobis]
        try:
            trace = float(np.trace(self.P))
            eigenvalues = np.linalg.eigvalsh(self.P)
            max_eig = float(np.max(eigenvalues))
            is_healthy = (
                trace < 10.0
                and not np.any(np.isnan(self.P))
                and not np.any(np.isinf(self.P))
            )
            self._ukf_healthy = bool(is_healthy)
            health_msg = Float64MultiArray()
            health_msg.data = [trace, 1.0 if is_healthy else 0.0,
                               max_eig, self._last_mahalanobis]
            self.health_pub.publish(health_msg)
        except Exception:
            pass

    def yaw_to_quat(self, yaw):
        """Convert yaw to quaternion"""
        q = Quaternion()
        q.w = math.cos(yaw/2.0)
        q.z = math.sin(yaw/2.0)
        q.x = 0.0; q.y = 0.0
        return q

def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
