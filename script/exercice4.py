import json
import pandas as pd
import numpy as np
import gurobipy as gp
from gurobipy import GRB

# Chargement des données JSON
with open("data/data/portfolio-example.json", "r") as f:
    data = json.load(f)

n = data["num_assets"]
sigma = np.array(data["covariance"])
mu = np.array(data["expected_return"])
mu_0 = data["target_return"]
k = data["portfolio_max_size"]

# --- Modèle Gurobi ---
with gp.Model("portfolio") as model:
    # 1️⃣ Variables
    x = model.addVars(n, lb=0, ub=1, name="x")        # fraction investie
    y = model.addVars(n, vtype=GRB.BINARY, name="y")  # indicateur sélectionné

    # 2️⃣ Objectif : minimiser le risque (quadratique)
    obj = gp.QuadExpr()
    for i in range(n):
        for j in range(n):
            obj.add(sigma[i, j] * x[i] * x[j])
    model.setObjective(obj, GRB.MINIMIZE)

    # 3️⃣ Contraintes
    model.addConstr(gp.quicksum(mu[i] * x[i] for i in range(n)) >= mu_0, name="return")
    model.addConstr(gp.quicksum(x[i] for i in range(n)) == 1, name="budget")
    model.addConstr(gp.quicksum(y[i] for i in range(n)) <= k, name="limit_assets")
    model.addConstrs((x[i] <= y[i] for i in range(n)), name="link")

    # 4️⃣ Résolution
    model.optimize()

    # 5️⃣ Résultats
    portfolio = [var.X for var in model.getVars() if "x" in var.VarName]
    risk = model.ObjVal
    expected_return = model.getRow(model.getConstrByName("return")).getValue()

    df = pd.DataFrame(
        data=portfolio + [risk, expected_return],
        index=[f"asset_{i}" for i in range(n)] + ["risk", "return"],
        columns=["Portfolio"],
    )

    print(df)
