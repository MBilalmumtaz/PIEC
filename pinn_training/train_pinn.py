#!/usr/bin/env python3
import argparse, os, numpy as np, torch, torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

class MLP(nn.Module):
    def __init__(self, in_dim=6, out_dim=2, hidden=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden//2),
            nn.Tanh(),
            nn.Linear(hidden//2, out_dim)
        )
    def forward(self,x):
        return self.net(x)

def load_csv(path):
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    # columns: t,x,y,yaw,v_cmd,omega_cmd,imu_wz,slope,roughness,sim_energy,sim_stability
    xs = data[:,1]; ys = data[:,2]; yaws = data[:,3]
    v = data[:,4]; slopes = data[:,7]; rough = data[:,8]
    energy = data[:,9]; stability = data[:,10]
    X = np.stack([xs, ys, yaws, v, slopes, rough], axis=1).astype(np.float32)
    Y = np.stack([energy, stability], axis=1).astype(np.float32)
    return X, Y

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', default='pinn_dataset.csv')
    parser.add_argument('--outdir', default='models_out')
    parser.add_argument('--epochs', type=int, default=120)
    parser.add_argument('--batch', type=int, default=256)
    parser.add_argument('--lr', type=float, default=1e-3)
    args = parser.parse_args()

    X, Y = load_csv(args.csv)
    # normalization
    meanX = X.mean(0); stdX = X.std(0) + 1e-9
    meanY = Y.mean(0); stdY = Y.std(0) + 1e-9
    Xn = (X - meanX) / stdX
    Yn = (Y - meanY) / stdY

    ds = TensorDataset(torch.from_numpy(Xn), torch.from_numpy(Yn))
    loader = DataLoader(ds, batch_size=args.batch, shuffle=True, drop_last=False)

    model = MLP(in_dim=X.shape[1], out_dim=Y.shape[1], hidden=128)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-6)
    loss_fn = nn.MSELoss()

    for epoch in range(args.epochs):
        total = 0.0
        for xb, yb in loader:
            xb = xb.to(device); yb = yb.to(device)
            opt.zero_grad()
            pred = model(xb)
            loss = loss_fn(pred, yb)
            loss.backward(); opt.step()
            total += loss.item() * xb.size(0)
        if epoch % 10 == 0 or epoch == args.epochs-1:
            print(f'Epoch {epoch:04d} loss {total/len(ds):.6f}')

    os.makedirs(args.outdir, exist_ok=True)
    save_path = os.path.join(args.outdir, 'pinn_energy.pt')
    ckpt = {
        'model_state': model.cpu().state_dict(),
        'scaler': {'mean': meanX.tolist(), 'std': stdX.tolist()},
        'Y_mean': meanY.tolist(), 'Y_std': stdY.tolist()
    }
    torch.save(ckpt, save_path)
    print("Saved checkpoint ->", save_path)

if __name__ == '__main__':
    main()

