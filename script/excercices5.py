import json
import gurobipy as gp
from gurobipy import GRB
from pathlib import Path

# ----- Load data from JSON -----
with open("data/lot_sizing_data.json", "r") as f:
    data = json.load(f)

name = data["name"]
H    = int(data["H"])
d    = [float(val) for val in data["demand"]]
c    = [float(val) for val in data["var_cost"]]
f    = [float(val) for val in data["setup_cost"]]
h    = [float(val) for val in data["hold_cost"]]
Qmin = float(data["Qmin"])
Qmax = float(data["Qmax"])
I0   = float(data["I0"])

# Basic validation
assert len(d) == H and len(c) == H and len(f) == H and len(h) == H
assert 0 <= Qmin <= Qmax

# ----- Build model -----
with gp.Env(empty=True) as env:
    env.setParam('OutputFlag', 0)
    env.start()

    with gp.Model(name, env=env) as model:

        # 1️⃣ Variables
        x = model.addVars(H, lb=0, name="x")          # quantité produite
        y = model.addVars(H, vtype=GRB.BINARY, name="y")  # production activée
        I = model.addVars(H, lb=0, name="I")          # stock en fin de période

        # 2️⃣ Fonction objectif
        model.setObjective(
            gp.quicksum(c[t]*x[t] + f[t]*y[t] + h[t]*I[t] for t in range(H)),
            GRB.MINIMIZE
        )

        # 3️⃣ Contraintes d’équilibre des stocks
        for t in range(H):
            if t == 0:
                model.addConstr(I0 + x[t] - d[t] == I[t], name=f"balance_{t}")
            else:
                model.addConstr(I[t-1] + x[t] - d[t] == I[t], name=f"balance_{t}")

        # 4️⃣ Contraintes de capacité et batch
        for t in range(H):
            model.addConstr(x[t] <= Qmax * y[t], name=f"maxcap_{t}")
            model.addConstr(x[t] >= Qmin * y[t], name=f"minbatch_{t}")

        # 5️⃣ Résolution
        model.optimize()

        # 6️⃣ Résultats
        if model.status == GRB.OPTIMAL:
            print(f"\n=== Optimal production plan: {name} ===")
            print(f"Total cost = {model.ObjVal:.2f}\n")
            print("Period | Demand | Prod | Setup | Stock")
            print("-----------------------------------------")
            for t in range(H):
                print(f"{t:6d} | {d[t]:7.1f} | {x[t].X:5.1f} | {int(y[t].X):5d} | {I[t].X:6.1f}")
        else:
            print("No optimal solution found.")
