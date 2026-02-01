#!/usr/bin/env python3
"""
Physics-Informed Neural Network (PINN) Training Script - ENHANCED VERSION
Now includes laser scan features for terrain and obstacle awareness.
"""
import argparse
import os
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from .pinn_model import PhysicsInformedPINN  # Import from same package

def load_csv(path):
    """Load dataset with enhanced features including laser scan data"""
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    
    # Extended dataset columns (14 total):
    # 0: t, 1: x, 2: y, 3: yaw, 4: v_cmd, 5: omega_cmd, 6: imu_wz,
    # 7: slope, 8: roughness, 9: obstacle_density, 10: clearance,
    # 11: terrain_type, 12: sim_energy, 13: sim_stability
    
    # Input features - now 10 dimensions including laser features
    xs = data[:, 1]          # x position
    ys = data[:, 2]          # y position
    yaws = data[:, 3]        # yaw angle
    velocities = data[:, 4]  # commanded velocity
    omegas = data[:, 5]      # commanded angular velocity
    slopes = data[:, 7]      # terrain slope
    roughness = data[:, 8]   # terrain roughness
    obstacle_density = data[:, 9]    # obstacle density from laser
    clearance = data[:, 10]          # average clearance from obstacles
    terrain_type = data[:, 11]       # terrain classification
    
    X = np.stack([
        xs, ys, yaws, velocities, omegas, 
        slopes, roughness, obstacle_density, clearance, terrain_type
    ], axis=1).astype(np.float32)
    
    # Outputs: energy and stability (ground truth from simulation)
    Y = np.stack([data[:, 12], data[:, 13]], axis=1).astype(np.float32)
    
    print(f"Loaded dataset with {X.shape[0]} samples, {X.shape[1]} input features")
    print(f"Feature ranges - Vel: [{velocities.min():.3f}, {velocities.max():.3f}], "
          f"Obstacle density: [{obstacle_density.min():.3f}, {obstacle_density.max():.3f}], "
          f"Clearance: [{clearance.min():.3f}, {clearance.max():.3f}]")
    
    return X, Y

def validate_dataset(X, Y):
    """Validate dataset quality and remove invalid samples"""
    # Remove NaN or infinite values
    valid_mask = ~(np.any(np.isnan(X), axis=1) | np.any(np.isnan(Y), axis=1) |
                   np.any(np.isinf(X), axis=1) | np.any(np.isinf(Y), axis=1))
    
    X_clean = X[valid_mask]
    Y_clean = Y[valid_mask]
    
    if len(X_clean) < len(X):
        print(f"Removed {len(X) - len(X_clean)} invalid samples")
    
    return X_clean, Y_clean

def main():
    parser = argparse.ArgumentParser(description='Train Enhanced Physics-Informed PINN')
    parser.add_argument('--csv', default='pinn_dataset.csv', help='Input CSV file')
    parser.add_argument('--outdir', default='models', help='Output directory')
    parser.add_argument('--epochs', type=int, default=200, help='Training epochs')
    parser.add_argument('--batch', type=int, default=64, help='Batch size')
    parser.add_argument('--lr', type=float, default=1e-3, help='Learning rate')
    parser.add_argument('--physics_weight', type=float, default=0.15, help='Physics loss weight')
    parser.add_argument('--validation_split', type=float, default=0.2, help='Validation split ratio')
    
    args = parser.parse_args()
    
    # Load and prepare data
    print(f"Loading dataset from {args.csv}...")
    X, Y = load_csv(args.csv)
    
    # Validate dataset
    X, Y = validate_dataset(X, Y)
    
    if len(X) == 0:
        print("❌ Error: No valid samples in dataset!")
        return
    
    # Split into train and validation sets
    n_samples = len(X)
    n_train = int(n_samples * (1 - args.validation_split))
    
    indices = np.random.permutation(n_samples)
    train_idx, val_idx = indices[:n_train], indices[n_train:]
    
    X_train, Y_train = X[train_idx], Y[train_idx]
    X_val, Y_val = X[val_idx], Y[val_idx]
    
    print(f"Training samples: {len(X_train)}, Validation samples: {len(X_val)}")
    
    # Normalization
    meanX = X_train.mean(axis=0)
    stdX = X_train.std(axis=0) + 1e-9
    meanY = Y_train.mean(axis=0)
    stdY = Y_train.std(axis=0) + 1e-9
    
    X_train_norm = (X_train - meanX) / stdX
    Y_train_norm = (Y_train - meanY) / stdY
    X_val_norm = (X_val - meanX) / stdX
    Y_val_norm = (Y_val - meanY) / stdY
    
    # Dataset and loader
    train_dataset = TensorDataset(torch.from_numpy(X_train_norm), torch.from_numpy(Y_train_norm))
    val_dataset = TensorDataset(torch.from_numpy(X_val_norm), torch.from_numpy(Y_val_norm))
    
    train_loader = DataLoader(train_dataset, batch_size=args.batch, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch, shuffle=False)
    
    # Model with enhanced input dimension (10 features)
    model = PhysicsInformedPINN(input_dim=X.shape[1], hidden_dim=128)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)
    
    # Optimizer with learning rate scheduler
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10
    )
    
    # Early stopping
    best_val_loss = float('inf')
    patience = 20
    patience_counter = 0
    
    # Training loop with physics-informed loss
    print(f"Training Enhanced Physics-Informed PINN for {args.epochs} epochs...")
    
    for epoch in range(args.epochs):
        # Training phase
        model.train()
        total_train_loss = 0.0
        total_data_loss = 0.0
        total_physics_loss = 0.0
        
        for x_batch, y_batch in train_loader:
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)
            
            optimizer.zero_grad()
            
            # Forward pass with physics constraints
            y_pred = model(x_batch, apply_physics_constraints=True)
            
            # Calculate combined loss
            data_loss = nn.MSELoss()(y_pred, y_batch)
            physics_loss = model.physics_loss(x_batch, y_pred, y_batch)
            
            loss = data_loss + args.physics_weight * physics_loss
            
            # Backward pass
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            
            total_train_loss += loss.item() * x_batch.size(0)
            total_data_loss += data_loss.item() * x_batch.size(0)
            total_physics_loss += physics_loss.item() * x_batch.size(0)
        
        avg_train_loss = total_train_loss / len(train_dataset)
        avg_data_loss = total_data_loss / len(train_dataset)
        avg_physics_loss = total_physics_loss / len(train_dataset)
        
        # Validation phase
        model.eval()
        total_val_loss = 0.0
        
        with torch.no_grad():
            for x_batch, y_batch in val_loader:
                x_batch = x_batch.to(device)
                y_batch = y_batch.to(device)
                
                y_pred = model(x_batch, apply_physics_constraints=True)
                data_loss = nn.MSELoss()(y_pred, y_batch)
                physics_loss = model.physics_loss(x_batch, y_pred, y_batch)
                loss = data_loss + args.physics_weight * physics_loss
                
                total_val_loss += loss.item() * x_batch.size(0)
        
        avg_val_loss = total_val_loss / len(val_dataset)
        
        # Update learning rate
        scheduler.step(avg_val_loss)
        
        # Early stopping check
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            patience_counter = 0
            # Save best model
            best_model_path = os.path.join(args.outdir, 'pinn_best.pt')
            os.makedirs(args.outdir, exist_ok=True)
            torch.save({
                'model_state': model.state_dict(),
                'epoch': epoch,
                'val_loss': avg_val_loss
            }, best_model_path)
        else:
            patience_counter += 1
        
        # Logging
        if epoch % 10 == 0 or epoch == args.epochs - 1 or patience_counter >= patience:
            print(f'Epoch {epoch:04d}: '
                  f'Train={avg_train_loss:.6f}, '
                  f'Data={avg_data_loss:.6f}, '
                  f'Physics={avg_physics_loss:.6f}, '
                  f'Val={avg_val_loss:.6f}, '
                  f'LR={optimizer.param_groups[0]["lr"]:.6f}')
        
        if patience_counter >= patience:
            print(f"Early stopping triggered at epoch {epoch}")
            break
    
    # Save final model
    os.makedirs(args.outdir, exist_ok=True)
    save_path = os.path.join(args.outdir, 'pinn_physics.pt')
    
    checkpoint = {
        'model_state': model.state_dict(),
        'scaler': {
            'mean': meanX.tolist(),
            'std': stdX.tolist(),
            'Y_mean': meanY.tolist(),
            'Y_std': stdY.tolist()
        },
        'input_dim': X.shape[1],
        'hidden_dim': 128,
        'epochs': epoch + 1,
        'physics_weight': args.physics_weight,
        'train_loss': avg_train_loss,
        'val_loss': avg_val_loss,
        'features': [
            'x', 'y', 'yaw', 'velocity', 'omega',
            'slope', 'roughness', 'obstacle_density', 'clearance', 'terrain_type'
        ]
    }
    
    torch.save(checkpoint, save_path)
    print(f"✅ Saved Enhanced Physics-Informed PINN to: {save_path}")
    print(f"Input features: {checkpoint['features']}")
    print(f"Final validation loss: {avg_val_loss:.6f}")

if __name__ == '__main__':
    main()
