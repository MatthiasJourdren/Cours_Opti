# Cours_Opti

# 🧮 Optimization Exercises with Gurobi (Python)

This repository gathers a series of optimization problems implemented in **Python using Gurobi Optimizer**.  
Each exercise explores a different class of mathematical programming problems — from linear knapsack to nonlinear robot control.  

All scripts were developed and tested under:
- **Python 3.11+**
- **Gurobi 12.0.3**
- **Academic license**

---

## 📘 Contents

| # | Title | Type | Description |
|---|-------|------|--------------|
| 1 | **Knapsack Problem** | Integer Linear Programming (ILP) | Maximize total item value within a capacity constraint. |
| 4 | **Portfolio Optimization** | Quadratic Programming (QP + binary) | Minimize portfolio risk while achieving a target expected return and limiting the number of active assets. |
| 5 | **Multi-Period Lot-Sizing** | Mixed-Integer Linear Programming (MILP) | Production planning with fixed setup costs, variable costs, and inventory holding costs. |
| 6 | **Bucket Design with Fixed Surface** | Nonlinear Programming (NLP) | Maximize the volume of a truncated cone (frustum) under a fixed total material surface. |
| 7 | **Custom Termination Criteria** | MIP + Callback | Stop optimization if MIPGap does not improve by more than ε over a given time window. |
| 8 | **Unit Commitment Problem** | MILP | Optimal on/off scheduling of thermal generators to meet energy demand while minimizing cost. |
| 9 | **Unit Commitment (Matrix API)** | Matrix API (MVar-based MILP) | Same as #8, rewritten using Gurobi’s matrix modeling API. |
| 10 | **Robotic Arm Kinematics** | Nonlinear Optimization (NLP) | Optimize a 2-joint robotic arm configuration to reach a target point while avoiding obstacles. |

---

## 🧠 Key Learning Points

### Linear & Integer Programming
- Model creation with `Model.addVars()`, `setObjective()`, and `addConstr()`.
- Use of **tupledict.prod()** for compact linear objectives.
- Handling of binary variables (`x[i] ∈ {0,1}`).

### Quadratic Programming
- Portfolio variance expressed as \( x' \Sigma x \).
- Risk-return tradeoff using linear and quadratic terms.

### Mixed-Integer Production Models
- Lot-sizing logic:
  - Inventory balance equations
  - Fixed setup costs via binary variables
  - Minimum and maximum batch sizes with indicator constraints.

### Nonlinear Optimization
- Use of `nlfunc` (e.g., `nl.sin`, `nl.cos`, `nl.sqrt`) to model nonlinear trigonometric or geometric relations.
- Setting `NonConvex = 2` to handle nonconvex models (Gurobi ≥ 10).
- Defining nonlinear constraints using auxiliary variables (`y = f(x)`).
- Visualizing solutions using **matplotlib** (e.g., bucket, robotic arm).

### Custom Control via Callbacks
- Implementing user-defined stopping criteria based on **MIPGap** evolution over time.
- Use of Python’s `functools.partial()` to pass custom data to callbacks.

### Matrix API (advanced)
- Using `addMVar()` and matrix expressions instead of Python loops.
- Clean, vectorized formulation of power system constraints.

---

## 🧩 Folder Structure

Cours/
│
├── ...
│
project/
│
├── ...
│
script/
│
├── exercice1_knapsack.py
├── exercice4_portfolio.py
├── exercice5_lot_sizing.py
├── exercice6_bucket_design.py
├── exercice7_callback_gap.py
├── exercice8_unit_commitment.py
├── exercice9_unit_commitment_matrix.py
├── exercice10_robot_arm.py
│
data/
│
├── portfolio-example.json
├── lot_sizing_data.json
├── mkp.mps.bz2
├── ...
│
images/
│
├── bucket_design.png
├── robot-arm.png
├── ...
│
sortie_txt/
│
├── infeasible_model.ilp
│
│
└── README.md