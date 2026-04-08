import numpy as np
import random as rnd
from pso import minimize as spso_minimize
from c_pso import minimize as cpso_minimize
from gl_pso import minimize as glbest_minimize
from i_pso import minimize as ipso_minimize
from LAIW_pso import minimize as laiw_minimize
from darp_cost_pso import darp_cost
from pso_parameter import NX, NY
from darp_cost_pso import darp_cost, get_outer_cells

# --- Variant registry ---------------------------------------------------------
variants = {
    "sPSO":   spso_minimize,
    "cPSO":   cpso_minimize,
    "GLBest": glbest_minimize,
    "IPSO":   ipso_minimize,
    "LAIW":   laiw_minimize,
}

NUM_RUNS = 10  # change to 100 for full experiment

# --- Run experiment -----------------------------------------------------------
all_results = {}

outer_cells = get_outer_cells()

for name, minimize_fn in variants.items():
    print(f"\n{'='*50}")
    print(f"Running {name} for {NUM_RUNS} runs...")
    print(f"{'='*50}")

    run_fitness  = []
    run_times    = []
    run_converge = []
    run_positions = []

    for run in range(NUM_RUNS):
        # Activate this to activate reproduciblity
        np.random.seed(run)
        rnd.seed(run)

        best_fitness, best_positions, pso_time, history, converge_iter = minimize_fn(
            costFunc=darp_cost,
            verbose=False
        )

        run_fitness.append(best_fitness)
        run_times.append(pso_time)
        run_converge.append(converge_iter)
        run_positions.append(best_positions)

        real_positions = [outer_cells[int(p)] for p in best_positions]
        real_positions_brac = [(p // NY, p % NY) for p in real_positions]

        print(f"  run{run+1:>3d}: fitness = {best_fitness:.2f}, " f"time = {pso_time:.3f} sec, converge_iter = {converge_iter}, " f" optimal intial positions = {real_positions_brac}")

    min_fitness = min(run_fitness)
    candidates = [
    i for i in range(NUM_RUNS)
    if abs(run_fitness[i] - min_fitness) < 1e-6
    ]

    best_run_idx = min(candidates, key=lambda i: (run_converge[i], run_times[i]))
    
    best_positions    = run_positions[best_run_idx]
    best_converge     = run_converge[best_run_idx]


    all_results[name] = {
        "Best Fitness (s)":     run_fitness[best_run_idx],
        "PSO Time Mean (s)":    np.mean(run_times),
        "PSO Time Std (s)":     np.std(run_times),
        "Converge Iter Mean":   np.mean(run_converge),
        "Converge Iter Std":    np.std(run_converge),
        "Best Run":             best_run_idx + 1,
        "Best Converge Iter":   best_converge,
        "Best Position":        best_positions,
    }


# --- Print summary ------------------------------------------------------------
metrics = [
    "Best Fitness (s)",
    "PSO Time Mean (s)",
    "PSO Time Std (s)",
    "Converge Iter Mean",
    "Converge Iter Std",
    "Best Run",
    "Best Converge Iter",
]

print(f"\n\n{'='*60}\nPSO PERFORMANCE SUMMARY\n{'='*60}")
print(f"{'Metric':<25}" + "".join(f"{n:>10}" for n in variants))

for metric in metrics:
    print(f"{metric:<25}" + "".join(
        f" {all_results[n][metric]:>10d}" if isinstance(all_results[n][metric], (int, np.integer))
        else f" {all_results[n][metric]:>10.3f}" for n in variants))

print(f"\n{'─'*60}\nBEST INITIAL POSITIONS PER VARIANT\n{'─'*60}")
for name, res in all_results.items():
    real_brac = [(outer_cells[int(p)] // NY, outer_cells[int(p)] % NY) for p in res["Best Position"]]
    print(f"  {name:<8}: {real_brac}  (run {res['Best Run']}, fitness = {res['Best Fitness (s)']:.2f})")