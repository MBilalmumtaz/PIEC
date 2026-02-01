#!/usr/bin/env python3
"""
Generate a small synthetic dataset for PINN training.
Outputs: pinn_dataset.csv with columns
t,x,y,yaw,v_cmd,omega_cmd,imu_wz,slope,roughness,sim_energy,sim_stability
"""
import csv, math, random
import numpy as np

N = 5000
outfile = 'pinn_dataset.csv'

def compute_sim_energy(v, slope, rough):
    # simple surrogate: E_rate = a*v^2 + b*slope + c*rough
    return 0.05 * v**2 + 2.0*abs(slope) + 1.0*rough

def compute_sim_stability(v, slope, rough):
    # stability score: higher is more stable (so we want larger values)
    base = max(0.1, 2.0 - 0.5*abs(slope) - 0.3*rough - 0.2*v)
    return base

with open(outfile, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t','x','y','yaw','v_cmd','omega_cmd','imu_wz','slope','roughness','sim_energy','sim_stability'])
    t = 0.0
    for i in range(N):
        x = random.uniform(-5,5)
        y = random.uniform(-5,5)
        yaw = random.uniform(-math.pi, math.pi)
        v = random.uniform(0.0, 1.2)
        omega = random.uniform(-0.8,0.8)
        imu_wz = omega + random.gauss(0,0.02)
        slope = random.uniform(-0.15, 0.15)   # simulated small slopes
        rough = random.uniform(0.0, 1.0)
        energy = compute_sim_energy(v, slope, rough) + random.gauss(0,0.01)
        stability = compute_sim_stability(v, slope, rough) + random.gauss(0,0.02)
        w.writerow([t, x, y, yaw, v, omega, imu_wz, slope, rough, energy, stability])
        t += 0.02

print("Saved", outfile)

