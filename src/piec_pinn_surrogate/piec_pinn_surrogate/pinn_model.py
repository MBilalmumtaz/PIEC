import torch
import torch.nn as nn
import numpy as np

class PhysicsInformedPINN(nn.Module):
    def __init__(self, input_dim=11, hidden_dim=128):
        """
        Enhanced PINN model with 11 input features including laser scan data

        Input features (11 dimensions):
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
        10: path length (m) over the 1-second window

        Output (2 dimensions):
        0: avg power (W)   [trained target]
        1: stability (0-1)
        """
        super().__init__()
        self.input_dim = input_dim

        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.fc4 = nn.Linear(hidden_dim // 2, 2)

        self.activation = nn.Tanh()
        self.dropout = nn.Dropout(0.1)

    def forward(self, x, apply_physics_constraints=True):
        h1 = self.activation(self.fc1(x))
        h1 = self.dropout(h1)

        h2 = self.activation(self.fc2(h1))
        h2 = self.dropout(h2)

        h3 = self.activation(self.fc3(h2))
        raw_output = self.fc4(h3)

        if apply_physics_constraints:
            # avg power must be non-negative
            power = torch.relu(raw_output[:, 0])
            # stability in [0,1]
            stability = torch.sigmoid(raw_output[:, 1])
            return torch.stack([power, stability], dim=1)

        return raw_output

    def physics_loss(self, x, y_pred, y_true=None):
        """
        Optional physics-inspired regularizer.

        IMPORTANT:
        This is now power-based (W), not energy (J).
        Keep weight small (e.g., 0.0 to 0.05) until you validate it helps.
        """
        loss = 0.0

        power = y_pred[:, 0]
        stability = y_pred[:, 1]

        v = torch.abs(x[:, 3])
        omega = torch.abs(x[:, 4])
        slope = torch.abs(x[:, 5])
        roughness = torch.clamp(x[:, 6], 0.0, 2.0)
        obstacle_density = torch.clamp(x[:, 7], 0.0, 1.0)
        clearance = torch.clamp(x[:, 8], 0.0, 10.0)

        # Very light "expected power" shape prior
        # P_expected ≈ P0 + a*v + b*|v*omega| + c*slope*v + d*obs*v
        P0 = 20.0
        a = 15.0
        b = 30.0
        c = 40.0
        d = 20.0

        expected_power = (
            P0 +
            a * v +
            b * (v * omega) +
            c * (slope * v) +
            d * (obstacle_density * v)
        )

        # Penalize large disagreement (scaled)
        power_loss = torch.mean(torch.abs(power - expected_power)) * 1e-4
        loss += power_loss

        # Stability prior (light)
        base_stability = 0.95
        st_expected = base_stability - (
            0.15 * (v / 1.5) +
            0.25 * (omega / 1.5) +
            0.35 * slope +
            0.40 * obstacle_density +
            0.15 * torch.relu(1.0 - clearance)
        )
        st_expected = torch.clamp(st_expected, 0.1, 1.0)

        st_loss = torch.mean(torch.abs(stability - st_expected)) * 1e-4
        loss += st_loss

        # Non-negativity (already enforced by relu in constrained forward, but keep tiny)
        loss += torch.mean(torch.relu(-power)) * 1e-6

        return loss

    def predict_with_breakdown(self, x):
        with torch.no_grad():
            prediction = self.forward(x, apply_physics_constraints=True)
            return prediction, torch.zeros_like(prediction)

def load_model(path, device='cpu'):
    try:
        ckpt = torch.load(path, map_location=device)

        input_dim = ckpt.get('input_dim', 11)
        hidden_dim = ckpt.get('hidden_dim', 128)

        model = PhysicsInformedPINN(input_dim=input_dim, hidden_dim=hidden_dim)

        if 'model_state' in ckpt:
            model.load_state_dict(ckpt['model_state'])
        else:
            model.load_state_dict(ckpt)

        model.eval()
        model.to(device)

        scaler = ckpt.get('scaler', None)

        print(f"Loaded PINN model with input_dim={input_dim}")
        return model, scaler

    except Exception as e:
        print(f"Error loading model: {e}")
        return PhysicsInformedPINN(), None
