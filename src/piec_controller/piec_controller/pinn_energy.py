def pinn_energy_cost(traj):
    energy = 0.0
    for i in range(1, len(traj)):
        dx = traj[i][0] - traj[i-1][0]
        dy = traj[i][1] - traj[i-1][1]
        energy += dx*dx + dy*dy
    return energy

