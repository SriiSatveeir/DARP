import importlib
import benchmark_pso
importlib.reload(benchmark_pso)
print("benchmark_pso.py loaded - UPDATED VERSION")
import numpy as np
import random as rnd
from benchmark_pso import minimize as spso_minimize
from benchmark_c_pso import minimize as cpso_minimize
from benchmark_gl_pso import minimize as glbest_minimize
from benchmark_i_pso_2008 import minimize as ipso_minimize
from benchmark_LAIW_pso import minimize as laiw_minimize
from benchmark_functions import sphere, sphere_bounds, sphere_dim, rosenbrock, rosenbrock_bounds, rosenbrock_dim, rastrigin, rastrigin_bounds, rastrigin_dim

# --- Variant registry ---------------------------------------------------------
variants = {
    "sPSO":   spso_minimize,
    # "cPSO":   cpso_minimize,
    # "GLBest": glbest_minimize,
    # "IPSO":   ipso_minimize,
    # "LAIW":   laiw_minimize,
}


# --- Experiment settings ------------------------------------------------------
NUM_RUNS = 10 # number of repeated runs per benchmark use 500

# func_name = "sphere"  # benchmark function name
# func_name = "rosenbrock"  # benchmark function name
func_name = "rastrigin"  # benchmark function name

num_particles = 80
maxiter = 1000

# --- Benchmark Functions ------------------------------------------------------
if func_name == "sphere":
    costFunc = sphere
    bounds = sphere_bounds
elif func_name == "rosenbrock":
    costFunc = rosenbrock
    bounds = rosenbrock_bounds
elif func_name == "rastrigin":
    costFunc = rastrigin
    bounds = rastrigin_bounds
else:
    raise ValueError("Invalid function")

# --- Run experiment -----------------------------------------------------------
all_results = {}

for name, minimize_fn in variants.items():
    print(f"\n{'='*50}")
    print(f"Running {name} for {NUM_RUNS} runs...")
    print(f"{'='*50}")

    # to this — shows the active function:
    print(f"Function: {func_name}")
    print(f"Bounds: {bounds[0]}")
    print(f"Dimensions: {len(bounds)}")

    run_fitness  = []
    run_times    = []
    gen_runs     = []

    for run in range(NUM_RUNS):
        # Activate this to activate reproduciblity
        rnd.seed(run)
        np.random.seed(run)


        best_fitness, best_positions, pso_time, history = minimize_fn(
            costFunc=costFunc,
            bounds=bounds,
            num_particles=num_particles,
            maxiter=maxiter,
            verbose=False
        )

        run_fitness.append(best_fitness)
        run_times.append(pso_time)

        thresholds = {
            "sphere":      1e-6,
            "rosenbrock":  6.0,
            "rastrigin":   1e-4,
        }

        threshold = thresholds[func_name]

        gen = next((i for i, f in enumerate(history) if f <= threshold), maxiter)
        gen_runs.append(gen)

        print(f"  run{run+1:>3d}: fitness = {best_fitness:.6e}, " f"time = {pso_time:.3f} sec")


    all_results[name] = {
        "Max Fitness":      max(run_fitness),
        "Min Fitness":      min(run_fitness),
        "Mean Fitness":     np.mean(run_fitness),
        "Median Fitness":   np.median(run_fitness),
        "Std Fitness":      np.std(run_fitness),
        "Mean Gen.":        np.mean(gen_runs), 
        "Mean PSO Time":    np.mean(run_times),
    }


# --- Print summary ------------------------------------------------------------
metrics = [
    "Max Fitness",
    "Min Fitness",
    "Mean Fitness",
    "Median Fitness",
    "Std Fitness",
    "Mean Gen.",
    "Mean PSO Time",
]

print(f"\n\n{'='*60}\nPSO PERFORMANCE SUMMARY\n{'='*60}")
print(f"{'Metric':<25}" + "".join(f"{n:>10}" for n in variants))

for metric in metrics:
    print(f"{metric:<25}" + "".join(
        f" {all_results[n][metric]:>15.4e}" for n in variants
    ))
