import math
import gurobipy as gp
from gurobipy import GRB, nlfunc as nl
import matplotlib.pyplot as plt

# --- Données ---
SURFACE = 1.0              # surface totale disponible (m²)
S = SURFACE / math.pi      # surface/pi (pour l'équation simplifiée)

m = gp.Model("bucket_frustum")
m.Params.NonConvex = 2
m.Params.OutputFlag = 1

# --- Variables principales ---
r = m.addVar(lb=0.0, ub=2.0, name="r")   # rayon bas
R = m.addVar(lb=0.0, ub=2.0, name="R")   # rayon haut
h = m.addVar(lb=0.0, ub=2.0, name="h")   # hauteur

# --- Variables auxiliaires ---
surf = m.addVar(lb=0.0, name="surf")     # surface totale / pi
V = m.addVar(lb=0.0, name="V")           # volume du seau

# --- Contrainte : surface totale = 1 m² ---
# surf = r² + (R + r)*sqrt((R - r)² + h²)
m.addConstr(
    surf == r*r + (R + r) * nl.sqrt((R - r)*(R - r) + h*h),
    name="surface_expr"
)
m.addConstr(surf == S, name="surface_eq")

# --- Contrainte : volume du tronc de cône ---
# V = (π/3) * h * (R² + Rr + r²)
m.addConstr(
    V == (math.pi/3.0) * h * (R*R + R*r + r*r),
    name="volume_expr"
)

# --- Objectif ---
m.setObjective(V, GRB.MAXIMIZE)

# --- Warm start ---
r.Start, R.Start, h.Start = 0.25, 0.45, 0.50

# --- Résolution ---
m.optimize()

# --- Résultats ---
if m.Status == GRB.OPTIMAL or m.Status == GRB.TIME_LIMIT:
    print(f"\n✅ Optimal solution found (status {m.Status})")
    print(f"r = {r.X:.6f} m")
    print(f"R = {R.X:.6f} m")
    print(f"h = {h.X:.6f} m")
    print(f"Volume = {V.X:.9f} m³")

    # Vérification de la surface
    Abot = math.pi * r.X*r.X
    Alat = math.pi * (R.X + r.X) * math.sqrt((R.X - r.X)**2 + h.X*h.X)
    print(f"A_bot = {Abot:.9f}, A_lat = {Alat:.9f}, total = {Abot+Alat:.9f} m²")

    # --- Visualisation du profil du seau ---
    fig, ax = plt.subplots(figsize=(5,6))

    # Coordonnées (profil en 2D)
    x_left = [-r.X, -R.X]
    x_right = [r.X, R.X]
    y = [0, h.X]

    # Tracé du contour du seau
    ax.plot(x_left, y, 'b-', linewidth=2)
    ax.plot(x_right, y, 'b-', linewidth=2)
    ax.plot([-r.X, r.X], [0, 0], 'b-', linewidth=2)  # fond

    # Remplissage (effet “seau”)
    ax.fill_betweenx(y, x_left, x_right, color='skyblue', alpha=0.4)

    # Détails
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel("Rayon (m)")
    ax.set_ylabel("Hauteur (m)")
    ax.set_title(
        f"Bucket Design (Fixed Surface = {SURFACE} m²)\n"
        f"r={r.X:.3f} m, R={R.X:.3f} m, h={h.X:.3f} m\n"
        f"Volume={V.X:.6f} m³"
    )
    ax.grid(True, linestyle=":")
    plt.tight_layout()
    plt.savefig("images/bucket_design.png", dpi=120)
    plt.close(fig)

    print("📊 Diagram saved as images/bucket_design.png")

else:
    print("❌ Optimization did not converge. Status:", m.Status)
