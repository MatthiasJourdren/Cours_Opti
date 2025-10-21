import math
import gurobipy as gp
from gurobipy import GRB, nlfunc as nl

# ---- Parameters ----
L1, L2 = 1.0, 0.8
x_star, y_star = 1.20, 0.60
xo, yo, r = 0.50, 0.00, 0.20

# Joint limits
theta1_min, theta1_max = -math.pi, math.pi
theta2_min, theta2_max = -0.75 * math.pi, 0.75 * math.pi

# Velocity limits (rad per step)
dtheta1_max = 0.25 * math.pi
dtheta2_max = 0.25 * math.pi

# Time discretization
T = 10  # number of time steps

# Initial joint configuration
theta1_init, theta2_init = 0.0, 0.0

# ---- Model ----
m = gp.Model("robot_arm_trajectory")
m.Params.NonConvex = 2

# ---- Variables ----
theta1 = m.addMVar(T, lb=theta1_min, ub=theta1_max, name="theta1")
theta2 = m.addMVar(T, lb=theta2_min, ub=theta2_max, name="theta2")

# End-effector position at each step
x = m.addMVar(T, lb=-GRB.INFINITY, name="x")
y = m.addMVar(T, lb=-GRB.INFINITY, name="y")

# Midpoint of link 1 for obstacle avoidance
xm = m.addMVar(T, lb=-GRB.INFINITY, name="xm")
ym = m.addMVar(T, lb=-GRB.INFINITY, name="ym")

# ---- Kinematic constraints ----
for t in range(T):
    m.addConstr(x[t] == L1 * nl.cos(theta1[t]) + L2 * nl.cos(theta1[t] + theta2[t]), name=f"kine_x_{t}")
    m.addConstr(y[t] == L1 * nl.sin(theta1[t]) + L2 * nl.sin(theta1[t] + theta2[t]), name=f"kine_y_{t}")
    m.addConstr(xm[t] == 0.5 * L1 * nl.cos(theta1[t]), name=f"mid_x_{t}")
    m.addConstr(ym[t] == 0.5 * L1 * nl.sin(theta1[t]), name=f"mid_y_{t}")

    # obstacle avoidance
    m.addConstr((xm[t] - xo) ** 2 + (ym[t] - yo) ** 2 >= r**2, name=f"obstacle_{t}")

# ---- Velocity limits ----
# initial step
m.addConstr(theta1[0] == theta1_init, name="init_theta1")
m.addConstr(theta2[0] == theta2_init, name="init_theta2")

for t in range(1, T):
    m.addConstr(theta1[t] - theta1[t - 1] <= dtheta1_max, name=f"vel_up1_{t}")
    m.addConstr(theta1[t - 1] - theta1[t] <= dtheta1_max, name=f"vel_down1_{t}")
    m.addConstr(theta2[t] - theta2[t - 1] <= dtheta2_max, name=f"vel_up2_{t}")
    m.addConstr(theta2[t - 1] - theta2[t] <= dtheta2_max, name=f"vel_down2_{t}")

# ---- Objective ----
# Minimize final distance to target + small penalty for total motion
m.setObjective(
    (x[T - 1] - x_star) ** 2 + (y[T - 1] - y_star) ** 2
    + 0.01 * gp.quicksum((theta1[t] - theta1[t - 1]) ** 2 + (theta2[t] - theta2[t - 1]) ** 2 for t in range(1, T)),
    GRB.MINIMIZE,
)

m.optimize()

# ---- Display results ----
if m.Status == GRB.OPTIMAL:
    print(f"\n✅ Optimal trajectory found, obj={m.ObjVal:.4f}")
    print("Step | θ1(rad) | θ2(rad) | x | y")
    print("-" * 40)
    for t in range(T):
        print(f"{t:3d}  | {theta1[t].X:7.3f} | {theta2[t].X:7.3f} | {x[t].X:5.2f} | {y[t].X:5.2f}")
else:
    print("Optimization failed:", m.Status)

# ---- Optional: plot the trajectory ----
import matplotlib.pyplot as plt

if m.Status == GRB.OPTIMAL:
    fig, ax = plt.subplots(figsize=(6, 6))

    # obstacle
    t_circle = [i * 2 * math.pi / 300 for i in range(301)]
    cx = [xo + r * math.cos(tt) for tt in t_circle]
    cy = [yo + r * math.sin(tt) for tt in t_circle]
    ax.plot(cx, cy, 'r--', label='obstacle')

    # target
    ax.scatter([x_star], [y_star], marker='x', s=80, label='target')

    # arm path
    arm_x = [x[t].X for t in range(T)]
    arm_y = [y[t].X for t in range(T)]
    ax.plot(arm_x, arm_y, 'bo-', label='end effector')

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-0.5, L1 + L2 + 0.2)
    ax.set_ylim(-0.5, L1 + L2 + 0.2)
    ax.grid(True, linestyle=':')
    ax.legend()
    ax.set_title(f"Robot Arm Trajectory (T={T})\nObj={m.ObjVal:.4f}")
    plt.savefig("images/robot-arm-trajectory.png", dpi=100, bbox_inches="tight")
    plt.close(fig)
