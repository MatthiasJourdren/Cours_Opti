from functools import partial
import time
import gurobipy as gp
from gurobipy import GRB


# Classe pour stocker l’état entre deux appels du callback
class CallbackData:
    def __init__(self):
        self.last_gap_change_time = None   # Temps de la dernière amélioration du gap
        self.last_gap = float('inf')       # Dernier gap connu


# Fonction callback
def callback(model, where, *, cbdata):
    if where != GRB.Callback.MIP:
        return

    # Nombre de solutions réalisables trouvées
    solcount = model.cbGet(GRB.Callback.MIP_SOLCNT)
    if solcount == 0:
        return  # Aucune solution encore

    best_bound = model.cbGet(GRB.Callback.MIP_OBJBND)
    best_obj   = model.cbGet(GRB.Callback.MIP_OBJBST)

    # Calcul du MIP gap (valeur absolue)
    if best_obj == 0:
        gap = float('inf')
    else:
        gap = abs(best_obj - best_bound) / max(1e-10, abs(best_obj))


    # Si c'est la première solution, on initialise les valeurs
    if cbdata.last_gap == float('inf'):
        cbdata.last_gap = gap
        cbdata.last_gap_change_time = time.time()
        return

    # Vérifier si le gap s'est amélioré de manière significative
    if cbdata.last_gap - gap > epsilon_to_compare_gap:
        cbdata.last_gap = gap
        cbdata.last_gap_change_time = time.time()
        return

    # Si aucune amélioration depuis 15 secondes, on arrête l’optimisation
    if time.time() - cbdata.last_gap_change_time > max_time_between_gap_updates:
        print(
            f"\n⏹ Terminating optimization: "
            f"MIPGap has not improved by more than {epsilon_to_compare_gap} "
            f"in {max_time_between_gap_updates} seconds."
        )
        model.terminate()


# ----- Charger et résoudre le modèle -----
with gp.read("data/mkp.mps.bz2") as model:
    # Paramètres du critère d'arrêt
    max_time_between_gap_updates = 15   # secondes
    epsilon_to_compare_gap = 1e-4

    # Données pour le callback
    callback_data = CallbackData()
    callback_func = partial(callback, cbdata=callback_data)

    # Résolution
    print("🔍 Starting optimization with custom termination criteria...")
    model.optimize(callback_func)

    # Résumé du résultat
    if model.status == GRB.Status.INTERRUPTED:
        print(f"\nOptimization stopped early by callback.")
    elif model.status == GRB.Status.OPTIMAL:
        print(f"\n✅ Optimal solution found with objective {model.ObjVal:.4f}")
    else:
        print(f"\nModel ended with status {model.status}")
