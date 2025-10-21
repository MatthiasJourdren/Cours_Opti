import gurobipy as gp
from gurobipy import GRB

# 24-hour load forecast (MW)
load_forecast = [
     4,  4,  4,  4,  4,  4,   6,   6,
    12, 12, 12, 12, 12,  4,   4,   4,
     4, 16, 16, 16, 16,  6.5, 6.5, 6.5,
]

# Solar energy forecast (MW)
solar_forecast = [
    0,   0,   0,   0,   0,   0,   0.5, 1.0,
    1.5, 2.0, 2.5, 3.5, 3.5, 2.5, 2.0, 1.5,
    1.0, 0.5, 0,   0,   0,   0,   0,   0,
]

nTimeIntervals = len(load_forecast)
thermal_units = ["gen1", "gen2", "gen3"]

# Costs (α + β*p + γ*p²), startup and shutdown costs
thermal_units_cost, a, b, c, sup_cost, sdn_cost = gp.multidict(
    {
        "gen1": [5.0, 0.5, 1.0, 2, 1],
        "gen2": [5.0, 0.5, 0.5, 2, 1],
        "gen3": [5.0, 3.0, 2.0, 2, 1],
    }
)

# Operating limits
thermal_units_limits, pmin, pmax = gp.multidict(
    {"gen1": [1.5, 5.0], "gen2": [2.5, 10.0], "gen3": [1.0, 3.0]}
)

# Initial on/off status
thermal_units_dyn_data, init_status = gp.multidict(
    {"gen1": [0], "gen2": [0], "gen3": [0]}
)


def show_results():
    obj_val_s = model.ObjVal
    print(f"\n💰 Total Cost = {round(obj_val_s, 2)}\n")
    print("%5s" % "time", end=" ")
    for t in range(nTimeIntervals):
        print("%4s" % t, end=" ")
    print("\n")

    for g in thermal_units:
        print("%5s" % g, end=" ")
        for t in range(nTimeIntervals):
            print("%4.1f" % thermal_units_out_power[g, t].X, end=" ")
        print("\n")

    print("%5s" % "Solar", end=" ")
    for t in range(nTimeIntervals):
        print("%4.1f" % solar_forecast[t], end=" ")
    print("\n")

    print("%5s" % "Load", end=" ")
    for t in range(nTimeIntervals):
        print("%4.1f" % load_forecast[t], end=" ")
    print("\n")


with gp.Env(empty=True) as env:
    env.setParam('OutputFlag', 0)
    env.start()

    with gp.Model("unit_commitment", env=env) as model:

        # --- 1️⃣ Variables ---
        thermal_units_out_power = model.addVars(
            thermal_units, range(nTimeIntervals),
            lb=0, name="p"
        )
        thermal_units_comm_status = model.addVars(
            thermal_units, range(nTimeIntervals),
            vtype=GRB.BINARY, name="u"
        )
        thermal_units_startup_status = model.addVars(
            thermal_units, range(nTimeIntervals),
            vtype=GRB.BINARY, name="v"
        )
        thermal_units_shutdown_status = model.addVars(
            thermal_units, range(nTimeIntervals),
            vtype=GRB.BINARY, name="w"
        )

        # --- 2️⃣ Objectif : minimiser le coût total ---
        obj_fun_expr = gp.QuadExpr()
        for t in range(nTimeIntervals):
            for g in thermal_units:
                obj_fun_expr.add(
                    c[g] * thermal_units_out_power[g, t] * thermal_units_out_power[g, t]
                    + b[g] * thermal_units_out_power[g, t]
                    + a[g] * thermal_units_comm_status[g, t]
                    + sup_cost[g] * thermal_units_startup_status[g, t]
                    + sdn_cost[g] * thermal_units_shutdown_status[g, t]
                )
        model.setObjective(obj_fun_expr, GRB.MINIMIZE)

        # --- 3️⃣ Contraintes de bilan de puissance ---
        for t in range(nTimeIntervals):
            model.addConstr(
                gp.quicksum(thermal_units_out_power[g, t] for g in thermal_units)
                + solar_forecast[t] == load_forecast[t],
                name=f"power_balance_{t}",
            )

        # --- 4️⃣ Contraintes logiques ---
        for g in thermal_units:
            for t in range(nTimeIntervals):
                if t == 0:
                    # Première période : dépend de l’état initial
                    model.addConstr(
                        thermal_units_comm_status[g, t] - init_status[g]
                        == thermal_units_startup_status[g, t]
                        - thermal_units_shutdown_status[g, t],
                        name=f"logical1_{g}_{t}",
                    )
                else:
                    model.addConstr(
                        thermal_units_comm_status[g, t]
                        - thermal_units_comm_status[g, t - 1]
                        == thermal_units_startup_status[g, t]
                        - thermal_units_shutdown_status[g, t],
                        name=f"logical1_{g}_{t}",
                    )

                # Interdiction de démarrer et s’arrêter en même temps
                model.addConstr(
                    thermal_units_startup_status[g, t]
                    + thermal_units_shutdown_status[g, t]
                    <= 1,
                    name=f"logical2_{g}_{t}",
                )

        # --- 5️⃣ Contraintes physiques (puissance min/max avec indicateurs) ---
        for g in thermal_units:
            for t in range(nTimeIntervals):
                # Si unité allumée : puissance min ≤ p ≤ max
                model.addGenConstrIndicator(
                    thermal_units_comm_status[g, t], True,
                    thermal_units_out_power[g, t] >= pmin[g],
                    name=f"pmin_{g}_{t}"
                )
                model.addGenConstrIndicator(
                    thermal_units_comm_status[g, t], True,
                    thermal_units_out_power[g, t] <= pmax[g],
                    name=f"pmax_{g}_{t}"
                )
                # Si unité éteinte : p = 0
                model.addGenConstrIndicator(
                    thermal_units_comm_status[g, t], False,
                    thermal_units_out_power[g, t] == 0,
                    name=f"p0_{g}_{t}"
                )

        # --- 6️⃣ Optimisation ---
        model.optimize()
        show_results()
