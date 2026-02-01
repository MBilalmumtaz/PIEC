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
        0: energy consumption rate (J/m)
        1: stability metric (0-1)
        """
        super().__init__()
        self.input_dim = input_dim
        
        # Enhanced network architecture with skip connections
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.fc4 = nn.Linear(hidden_dim // 2, 2)  # [energy_rate, stability]
        
        self.activation = nn.Tanh()
        self.dropout = nn.Dropout(0.1)
        
    def forward(self, x, apply_physics_constraints=True):
        """Forward pass with enhanced physics constraints including obstacle awareness"""
        # Neural network forward pass
        h1 = self.activation(self.fc1(x))
        h1 = self.dropout(h1)
        
        h2 = self.activation(self.fc2(h1))
        h2 = self.dropout(h2)
        
        h3 = self.activation(self.fc3(h2))
        
        raw_output = self.fc4(h3)
        
        if apply_physics_constraints:
            # Enhanced physics constraints with obstacle awareness
            
            # Extract features
            velocity = x[:, 3]          # linear velocity
            omega = x[:, 4]             # angular velocity
            slope = x[:, 5]             # terrain slope
            roughness = x[:, 6]         # terrain roughness
            obstacle_density = x[:, 7]  # obstacle density from laser
            clearance = x[:, 8]         # clearance from obstacles
            
            # Energy must be non-negative (conservation of energy)
            energy = torch.relu(raw_output[:, 0])
            
            # Stability must be between 0 and 1
            stability = torch.sigmoid(raw_output[:, 1])
            
            # Enhanced physics-informed energy correction
            # 1. Kinetic energy term
            kinetic_term = 0.5 * velocity**2  # ½mv² (assuming m=1 for normalization)
            
            # 2. Turning energy term
            turning_term = 0.2 * torch.abs(omega)
            
            # 3. Terrain effects
            slope_term = torch.abs(slope) * velocity * 2.0
            
            # 4. Friction effects
            friction_term = 0.1 * roughness * velocity
            
            # 5. Obstacle avoidance energy penalty
            obstacle_penalty = 0.3 * obstacle_density * velocity
            
            # 6. Clearance bonus (more clearance = less energy for avoidance)
            clearance_bonus = 0.1 * torch.relu(2.0 - clearance) / 2.0
            
            # Combined physics-informed energy
            physics_energy = (kinetic_term + turning_term + slope_term + 
                            friction_term + obstacle_penalty - clearance_bonus)
            
            # Blend neural network prediction with physics (adaptive blending)
            # More physics influence when obstacles are present
            physics_weight = 0.3 + 0.3 * obstacle_density
            nn_weight = 1.0 - physics_weight
            
            blended_energy = nn_weight * energy + physics_weight * physics_energy
            
            # Enhanced stability with obstacle awareness
            # Stability decreases with obstacles and high speeds
            stability_reduction = (
                0.2 * torch.abs(velocity) +          # Speed penalty
                0.3 * torch.abs(slope) +             # Slope penalty  
                0.4 * obstacle_density +             # Obstacle penalty
                0.2 * (1.0 - torch.sigmoid(clearance - 1.0))  # Low clearance penalty
            )
            
            # Apply stability reduction
            adjusted_stability = stability * (1.0 - stability_reduction)
            adjusted_stability = torch.clamp(adjusted_stability, 0.1, 1.0)
            
            return torch.stack([blended_energy, adjusted_stability], dim=1)
        
        return raw_output
    
    def physics_loss(self, x, y_pred, y_true=None):
        """Enhanced physics-informed loss terms including obstacle constraints"""
        loss = 0.0
        
        # Extract features
        energy = y_pred[:, 0]
        stability = y_pred[:, 1]
        
        velocity = x[:, 3]
        omega = x[:, 4]
        slope = x[:, 5]
        roughness = x[:, 6]
        obstacle_density = x[:, 7]
        clearance = x[:, 8]
        
        # 1. Non-negativity constraint for energy
        non_neg_loss = torch.mean(torch.relu(-energy)) * 0.1
        
        # 2. Stability bounds (0-1)
        bounds_loss = torch.mean(torch.relu(stability - 1) + torch.relu(-stability)) * 0.1
        
        # 3. Energy conservation with obstacle awareness
        # Base physics model
        expected_energy = (
            0.5 * velocity**2 +                    # Kinetic energy
            0.2 * torch.abs(omega) +               # Turning energy
            2.0 * torch.abs(slope) * velocity +    # Slope energy
            0.1 * roughness * velocity +           # Friction
            0.3 * obstacle_density * velocity      # Obstacle avoidance
        )
        conservation_loss = torch.mean((energy - expected_energy)**2) * 0.01
        
        # 4. Enhanced stability constraints
        expected_stability = 1.0 - (
            0.2 * torch.abs(velocity) +            # Speed effect
            0.3 * torch.abs(slope) +              # Slope effect
            0.4 * obstacle_density +              # Obstacle effect
            0.2 * torch.sigmoid(1.0 - clearance)  # Clearance effect
        )
        stability_loss = torch.mean((stability - expected_stability)**2) * 0.01
        
        # 5. Clearance constraint (prefer higher clearance for safety)
        clearance_loss = torch.mean(torch.relu(0.5 - clearance)) * 0.05
        
        # 6. Obstacle density smoothness (avoid sudden changes)
        # This requires temporal data, simplified here
        obstacle_smoothness = torch.mean(torch.abs(obstacle_density)) * 0.01
        
        total_physics_loss = (
            non_neg_loss + bounds_loss + conservation_loss + 
            stability_loss + clearance_loss + obstacle_smoothness
        )
        
        # Add data loss if ground truth is provided
        if y_true is not None:
            mse_loss = nn.MSELoss()(y_pred, y_true)
            total_physics_loss += mse_loss
        
        return total_physics_loss
    
    def predict_energy_breakdown(self, x):
        """Predict and return energy breakdown for analysis"""
        with torch.no_grad():
            # Get base prediction
            prediction = self.forward(x, apply_physics_constraints=True)
            
            # Extract features
            velocity = x[:, 3]
            omega = x[:, 4]
            slope = x[:, 5]
            roughness = x[:, 6]
            obstacle_density = x[:, 7]
            clearance = x[:, 8]
            
            # Calculate energy components
            kinetic_energy = 0.5 * velocity**2
            turning_energy = 0.2 * torch.abs(omega)
            slope_energy = 2.0 * torch.abs(slope) * velocity
            friction_energy = 0.1 * roughness * velocity
            obstacle_energy = 0.3 * obstacle_density * velocity
            clearance_energy = -0.1 * torch.relu(2.0 - clearance) / 2.0  # Negative = bonus
            
            energy_components = torch.stack([
                kinetic_energy, turning_energy, slope_energy,
                friction_energy, obstacle_energy, clearance_energy
            ], dim=1)
            
            return prediction, energy_components

def load_model(path, device='cpu'):
    """Load trained enhanced PINN model with physics constraints"""
    try:
        ckpt = torch.load(path, map_location=device)
        
        model = PhysicsInformedPINN(
            input_dim=ckpt.get('input_dim', 10),
            hidden_dim=ckpt.get('hidden_dim', 128)
        )
        
        model.load_state_dict(ckpt['model_state'])
        model.eval()
        
        scaler = ckpt.get('scaler', None)
        
        print(f"Loaded enhanced PINN model with {ckpt.get('input_dim', 10)} input features")
        if 'features' in ckpt:
            print(f"Features: {ckpt['features']}")
        
        return model, scaler
        
    except Exception as e:
        print(f"Error loading model: {e}")
        print("Falling back to default model")
        return PhysicsInformedPINN(), None
