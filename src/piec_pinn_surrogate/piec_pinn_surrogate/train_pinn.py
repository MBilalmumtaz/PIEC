#!/usr/bin/env python3
"""
Train PINN on recorded 1-second-window dataset.

Dataset columns (pinn_dataset_1s.csv):
0 t_end
1..11 X (11 inputs)
12..13 Y (2 targets): avg_power_W, stability
"""
import argparse
import os
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from sklearn.model_selection import train_test_split
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from pinn_model import PhysicsInformedPINN


def load_csv(path):
    print(f"Loading dataset from {path}...")
    data = np.loadtxt(path, delimiter=',', skiprows=1)

    X = data[:, 1:12].astype(np.float32)
    Y = data[:, 12:14].astype(np.float32)

    valid_mask = ~(np.any(np.isnan(X), axis=1) | np.any(np.isnan(Y), axis=1) |
                   np.any(np.isinf(X), axis=1) | np.any(np.isinf(Y), axis=1))
    X = X[valid_mask]
    Y = Y[valid_mask]

    print(f"Loaded {len(X)} samples")
    return X, Y


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', default='pinn_dataset_1s.csv')
    p.add_argument('--outdir', default='models')
    p.add_argument('--epochs', type=int, default=200)
    p.add_argument('--batch', type=int, default=64)
    p.add_argument('--lr', type=float, default=1e-3)
    p.add_argument('--physics_weight', type=float, default=0.0)
    p.add_argument('--validation_split', type=float, default=0.2)
    p.add_argument('--hidden_dim', type=int, default=128)
    p.add_argument('--patience', type=int, default=20)
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    X, Y = load_csv(args.csv)
    if len(X) == 0:
        print("No valid samples.")
        return

    X_train, X_val, Y_train, Y_val = train_test_split(
        X, Y, test_size=args.validation_split, random_state=42
    )

    meanX = X_train.mean(axis=0)
    stdX = X_train.std(axis=0) + 1e-9
    meanY = Y_train.mean(axis=0)
    stdY = Y_train.std(axis=0) + 1e-9

    X_train_n = (X_train - meanX) / stdX
    X_val_n = (X_val - meanX) / stdX
    Y_train_n = (Y_train - meanY) / stdY
    Y_val_n = (Y_val - meanY) / stdY

    train_ds = TensorDataset(torch.from_numpy(X_train_n).float(),
                             torch.from_numpy(Y_train_n).float())
    val_ds = TensorDataset(torch.from_numpy(X_val_n).float(),
                           torch.from_numpy(Y_val_n).float())

    train_loader = DataLoader(train_ds, batch_size=args.batch, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch, shuffle=False)

    model = PhysicsInformedPINN(input_dim=X.shape[1], hidden_dim=args.hidden_dim)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)

    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, mode='min', factor=0.5, patience=10)
    mse = nn.MSELoss()

    best_val = float('inf')
    patience = 0
    best_state = None

    for epoch in range(args.epochs):
        model.train()
        tr_loss = 0.0

        for xb, yb in train_loader:
            xb = xb.to(device)
            yb = yb.to(device)
            opt.zero_grad()

            y_pred_raw = model(xb, apply_physics_constraints=False)
            data_loss = mse(y_pred_raw, yb)

            loss = data_loss
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()

            tr_loss += loss.item() * xb.size(0)

        tr_loss /= len(train_ds)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb = xb.to(device)
                yb = yb.to(device)
                y_pred_raw = model(xb, apply_physics_constraints=False)
                val_loss += mse(y_pred_raw, yb).item() * xb.size(0)
        val_loss /= len(val_ds)

        scheduler.step(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            best_state = model.state_dict()
            patience = 0
        else:
            patience += 1

        if epoch % 10 == 0 or epoch == args.epochs - 1:
            lr = opt.param_groups[0]['lr']
            print(f"Epoch {epoch:04d} train={tr_loss:.6f} val={val_loss:.6f} lr={lr:.2e}")

        if patience >= args.patience:
            print(f"Early stopping at epoch {epoch}")
            break

    if best_state is not None:
        model.load_state_dict(best_state)

    save_path = os.path.join(args.outdir, 'pinn_physics.pt')
    torch.save({
        'model_state': model.state_dict(),
        'scaler': {
            'mean': meanX.tolist(),
            'std': stdX.tolist(),
            'Y_mean': meanY.tolist(),
            'Y_std': stdY.tolist()
        },
        'input_dim': X.shape[1],
        'hidden_dim': args.hidden_dim,
        'features': [
            'x', 'y', 'yaw', 'v', 'omega',
            'slope', 'roughness', 'obstacle_density',
            'clearance', 'terrain_type', 'path_length'
        ],
        'targets': ['avg_power_W', 'stability']
    }, save_path)

    print(f"Saved model to {save_path}")


if __name__ == '__main__':
    main()
