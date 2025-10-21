import numpy as np
import gurobipy as gp
from gurobipy import GRB

def generate_knapsack(num_items):
    # Fix seed value
    rng = np.random.default_rng(seed=0)
    # Item values, weights
    values = rng.uniform(low=1, high=25, size=num_items)
    weights = rng.uniform(low=5, high=100, size=num_items)
    # Knapsack capacity
    capacity = 0.7 * weights.sum()

    return values, weights, capacity


def solve_knapsack_model(values, weights, capacity):
    num_items = len(values)
    
    # Convert to dictionaries for Gurobi
    items = range(num_items)
    val_dict = {i: values[i] for i in items}
    wgt_dict = {i: weights[i] for i in items}

    with gp.Env(empty=True) as env:
        env.setParam('OutputFlag', 0)  # Désactive l'affichage
        env.start()
        
        with gp.Model(name="knapsack", env=env) as model:
            # 1️⃣ Variables binaires x_i ∈ {0,1}
            x = model.addVars(items, vtype=GRB.BINARY, name="x")

            # 2️⃣ Fonction objectif : max Σ r_i x_i
            model.setObjective(x.prod(val_dict), GRB.MAXIMIZE)

            # 3️⃣ Contrainte de capacité : Σ c_i x_i ≤ C
            model.addConstr(x.prod(wgt_dict) <= capacity, name="capacity")

            # 4️⃣ Résolution
            model.optimize()

            # 5️⃣ Affichage des résultats
            if model.status == GRB.OPTIMAL:
                selected_items = [i for i in items if x[i].X > 0.5]
                total_value = sum(values[i] for i in selected_items)
                total_weight = sum(weights[i] for i in selected_items)
                print(f"Optimal value: {total_value:.2f}")
                print(f"Total weight: {total_weight:.2f} / {capacity:.2f}")
                print(f"Items selected: {selected_items[:10]} ... ({len(selected_items)} items)")
            else:
                print("No optimal solution found.")


# Test du modèle
data = generate_knapsack(10000)
solve_knapsack_model(*data)
