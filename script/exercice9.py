import gurobipy as gp
from gurobipy import GRB
import numpy as np

# 24h load and solar forecasts
load_forecast = np.array([
     4,  4,  4,  4,  4,  4,   6,   6,
    12, 12, 12, 12, 12,  4,   4,   4,
     4, 16, 16, 16, 16,  6.5, 6.5, 6.5,
])

solar_forecast = np.array([
    0,   0,   0,   0,   0,   0,   0.5, 1.0,
    1.5, 2.0, 2.5, 3.5, 3.5, 2.5, 2.0, 1.5,
    1.0, 0.5, 0,   0,   0,   0,   0,   0,
])

# Sets
T = len(load_forecast)
I = 3  # gen1, gen2, gen3
gens = range(I)
times = range(T)

# Parameters (arrays indexed by generator)
a  = np.array([5.0, 5.0, 5.0])
b  = np.array([0.5, 0.5, 3.0])
c  = np.array([1.0, 0.5, 2.0])
sup_cost = np.array([2, 2, 2])
sdn_cost = np.array([1, 1, 1])
pmin = np.array([1.5, 2.5, 1.0])
pmax = np.array([5.0, 10.0, 3.0])
init_status = np.array([0, 0, 0])

# Model
with gp.Env(empty=True) as env:
    env.setParam('OutputFlag', 0)
    env.start()

    with gp.Model("unit_commitment_matrix_api", env=env) as model:

        # --- 1️⃣ Variables matricielles ---
        p = model.addMVar((I, T), lb=0.0, name="p")
        u = model.addMVar((I, T), vtype=GRB.BINARY, name="u")
        v = model.addMVar((I, T), vtype=GRB.BINARY, name="v")
        w = model.addMVar((I, T), vtype=GRB.BINARY, name="w")

        # --- 2️⃣ Objectif (vectorisé) ---
        # somme_i somme_t [ c_i p_it^2 + b_i p_it + a_i u_it + sup_i v_it + sdn_i w_it ]
        cost_quadratic = gp.QuadExpr()
        for i in gens:
            cost_quadratic += (
                c[i] * (p[i, :] @ p[i, :]) +
                b[i] * gp.quicksum(p[i, :]) +
                a[i] * gp.quicksum(u[i, :]) +
                sup_cost[i] * gp.quicksum(v[i, :]) +
                sdn_cost[i] * gp.quicksum(w[i, :])
            )
        model.setObjective(cost_quadratic, GRB.MINIMIZE)

        # --- 3️⃣ Power balance: (p.sum(axis=0) + solar == load) ---
        model.addConstr(p.sum(axis=0) + solar_forecast == load_forecast, name="power_balance")

        # --- 4️⃣ Logical constraints (vectorisé sauf t=0) ---
        # For t = 0
        model.addConstr(
            u[:, 0] - init_status == v[:, 0] - w[:, 0],
            name="logical_init"
        )

        # For t >= 1
        model.addConstr(u[:, 1:] - u[:, :-1] == v[:, 1:] - w[:, 1:], name="logical_dyn")

        # No simultaneous startup/shutdown
        model.addConstr(v + w <= 1, name="logical_excl")

        # --- 5️⃣ Physical constraints (indicators: need loops) ---
        for i in gens:
            for t in times:
                # If on: p >= pmin
                model.addGenConstrIndicator(u[i, t], True, p[i, t] >= pmin[i], name=f"pmin_{i}_{t}")
                # If on: p <= pmax
                model.addGenConstrIndicator(u[i, t], True, p[i, t] <= pmax[i], name=f"pmax_{i}_{t}")
                # If off: p = 0
                model.addGenConstrIndicator(u[i, t], False, p[i, t] == 0, name=f"p0_{i}_{t}")

        # --- 6️⃣ Solve ---
        model.optimize()

        # --- 7️⃣ Résultats ---
        print(f"\n✅ Optimal total cost = {model.ObjVal:.2f}\n")
        print("time | load | solar | gen1 | gen2 | gen3")
        print("-" * 45)
        for t in times:
            total_gen = sum(p[i, t].X for i in gens)
            print(f"{t:4d} | {load_forecast[t]:5.1f} | {solar_forecast[t]:5.1f} | "
                  + " ".join(f"{p[i, t].X:5.1f}" for i in gens))
