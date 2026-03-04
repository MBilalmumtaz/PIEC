import torch
import torch.nn as nn
import numpy as np

class PhysicsInformedPINN(nn.Module):
    def __init__(self, input_dim=10, hidden_dim=128):
        """
        Enhanced PINN model with 10 input features including laser scan data
        
        Input features (10 dimensions):
        0: x position
        1: y position  
        2: yaw angle
        3: linear velocity
        4: angular velocity
        5: terrain slope
        6: terrain roughness
        7: obstacle density (from laser)
        8: clearance (from laser)
        9: terrain type
        
        Output (2 dimensions):
        0: energy consumption (J)
        1: stability metric (0-1)
        """
        super().__init__()
        self.input_dim = input_dim
        
        # Enhanced network architecture
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.fc4 = nn.Linear(hidden_dim // 2, 2)  # [energy, stability]
        
        self.activation = nn.Tanh()
        self.dropout = nn.Dropout(0.1)
        
    def forward(self, x, apply_physics_constraints=True):
        """
        Forward pass with physics constraints
        
        Args:
            x: Input tensor [batch_size, input_dim] (normalized)
            apply_physics_constraints: Whether to apply physics constraints
        
        Returns:
            y: Output tensor [batch_size, 2] (energy, stability)
        """
        # Neural network forward pass
        h1 = self.activation(self.fc1(x))
        h1 = self.dropout(h1)
        
        h2 = self.activation(self.fc2(h1))
        h2 = self.dropout(h2)
        
        h3 = self.activation(self.fc3(h2))
        
        raw_output = self.fc4(h3)
        
        if apply_physics_constraints:
            # Physics constraints
            
            # Energy must be non-negative (ReLU)
            energy = torch.relu(raw_output[:, 0])
            
            # Stability must be between 0 and 1 (sigmoid)
            stability = torch.sigmoid(raw_output[:, 1])
            
            # Note: The physics constraints are applied in training via physics_loss
            # Here we just ensure valid ranges
            return torch.stack([energy, stability], dim=1)
        
        return raw_output
    
    def physics_loss(self, x, y_pred, y_true=None):
        """
        Enhanced physics-informed loss terms - ULTRA SCALED DOWN VERSION
        """
        loss = 0.0
        
        # Extract features (denormalized values)
        energy = y_pred[:, 0]
        stability = y_pred[:, 1]
        
        velocity = torch.abs(x[:, 3])          # Linear velocity
        omega = torch.abs(x[:, 4])              # Angular velocity
        slope = torch.abs(x[:, 5])               # Terrain slope
        roughness = x[:, 6]                      # Terrain roughness
        obstacle_density = x[:, 7]                # Obstacle density
        clearance = x[:, 8]                       # Clearance
        
        # Robot physical parameters
        mass = 50.0      # kg
        g = 9.81         # m/s²
        
        # 1. Energy should increase with challenging terrain
        # Physics-based expected energy (Joules) - EXTREME SCALING
        kinetic = 0.5 * mass * velocity**2 / 10000.0  # Divided by 10000
        turning = 0.1 * mass * omega * velocity / 10000.0
        slope_energy = mass * g * slope * 0.1 / 10000.0
        friction = 0.05 * mass * g * roughness * velocity / 10000.0
        obstacle_energy = 0.3 * mass * obstacle_density * velocity**2 / 10000.0
        
        expected_energy = kinetic + turning + slope_energy + friction + obstacle_energy
        
        # Energy conservation loss - TINY WEIGHT
        energy_diff = torch.abs(energy/10000.0 - expected_energy)
        energy_loss = torch.mean(energy_diff) * 0.00001  # Even smaller!
        loss += energy_loss
        
        # 2. Stability should decrease with risk factors
        base_stability = 0.9
        speed_penalty = 0.2 * (velocity / 2.0)
        turn_penalty = 0.3 * (omega / 1.0)
        slope_penalty = 0.3 * slope
        roughness_penalty = 0.2 * roughness
        obstacle_penalty = 0.5 * obstacle_density
        clearance_penalty = 0.2 * torch.relu(1.0 - clearance)
        
        expected_stability = base_stability - (
            speed_penalty + turn_penalty + slope_penalty + 
            roughness_penalty + obstacle_penalty + clearance_penalty
        )
        expected_stability = torch.clamp(expected_stability, 0.1, 1.0)
        
        # Stability loss - TINY WEIGHT
        stability_diff = torch.abs(stability - expected_stability)
        stability_loss = torch.mean(stability_diff) * 0.00001
        loss += stability_loss
        
        # 3. Non-negativity constraint - minimal
        non_neg_loss = torch.mean(torch.relu(-energy)) * 0.0001
        loss += non_neg_loss
        
        # Add data loss if ground truth provided - use full weight
        if y_true is not None:
            mse_loss = nn.MSELoss()(y_pred, y_true)
            loss += mse_loss  # Full data loss - this is what matters!
        
        return loss
    
    def predict_with_breakdown(self, x):
        """
        Predict and return energy breakdown for analysis
        
        Args:
            x: Input tensor [batch_size, input_dim] (normalized)
        
        Returns:
            prediction: [energy, stability]
            components: Energy breakdown components
        """
        with torch.no_grad():
            # Get base prediction
            prediction = self.forward(x, apply_physics_constraints=True)
            
            # Extract features (denormalized version would need scaler)
            # For analysis, we just return the prediction and placeholder components
            return prediction, torch.zeros_like(prediction)

def load_model(path, device='cpu'):
    """
    Load trained enhanced PINN model with physics constraints
    
    Args:
        path: Path to model checkpoint
        device: Device to load model on
    
    Returns:
        model: Loaded PINN model
        scaler: Scaler dictionary for normalization
    """
    try:
        ckpt = torch.load(path, map_location=device)
        
        # Get model dimensions from checkpoint
        input_dim = ckpt.get('input_dim', 10)
        hidden_dim = ckpt.get('hidden_dim', 128)
        
        # Create model
        model = PhysicsInformedPINN(
            input_dim=input_dim,
            hidden_dim=hidden_dim
        )
        
        # Load state dict
        if 'model_state' in ckpt:
            model.load_state_dict(ckpt['model_state'])
        else:
            model.load_state_dict(ckpt)
        
        model.eval()
        model.to(device)
        
        # Get scaler
        scaler = ckpt.get('scaler', None)
        
        print(f"✅ Loaded enhanced PINN model with {input_dim} input features")
        if 'features' in ckpt:
            print(f"   Features: {ckpt['features']}")
        if scaler:
            print(f"   Scaler: Y_mean={scaler.get('Y_mean', [0,0])}")
        
        return model, scaler
        
    except Exception as e:
        print(f"❌ Error loading model: {e}")
        print("   Falling back to default model")
        return PhysicsInformedPINN(), None
