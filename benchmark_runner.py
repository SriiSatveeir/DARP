import numpy as np
from benchmark_functions import BF
from benchmark_pso import minimize  # your standard PSO function
import numpy as np
import random as rnd
from pso import minimize as spso_minimize
from c_pso import minimize as cpso_minimize
from gl_pso import minimize as glbest_minimize
from i_pso import minimize as ipso_minimize
from LAIW_pso import minimize as laiw_minimize

# --- Variant registry ---------------------------------------------------------
variants = {
    "sPSO":   spso_minimize,
    "cPSO":   cpso_minimize,
    "GLBest": glbest_minimize,
    "IPSO":   ipso_minimize,
    "LAIW":   laiw_minimize,
}


# --- Experiment settings ------------------------------------------------------
NUM_RUNS = 10  # number of repeated runs per benchmark
func_name = "sphere"  # benchmark function name
bf = BF(func_name)
costFunc = bf.get_function()  # callable fitness function

lower_bounds = np.array(bf.FUNCTION_RANGES[func_name]["lower"])
upper_bounds = np.array(bf.FUNCTION_RANGES[func_name]["upper"])
bounds = list(zip(lower_bounds, upper_bounds))

num_particles = 30
maxiter = 100


# --- Run experiment -----------------------------------------------------------
all_results = {}

for name, minimize_fn in variants.items():
    print(f"\n{'='*50}")
    print(f"Running {name} for {NUM_RUNS} runs...")
    print(f"{'='*50}")

    run_fitness  = []
    run_times    = []
    run_converge = []

    for run in range(NUM_RUNS):
        # Activate this to activate reproduciblity
        np.random.seed(run)
        rnd.seed(run)

        best_value, pso_time, history = minimize_fn(
            costFunc=costFunc,
            bounds=bounds,
            num_particles=num_particles,
            maxiter=maxiter,
            verbose=False
        )

        run_fitness.append(best_value)
        run_times.append(pso_time)

        print(f"  run{run+1:>3d}: fitness = {best_value:.2f}, " f"time = {pso_time:.3f} sec")


    all_results[name] = {
        "Max Fitness":      max(run_fitness),
        "Min Fitness":      min(run_fitness),
        "Mean Fitness":     np.mean(run_fitness),
        "Median Fitness":   np.median(run_fitness),
        "Std Fitness":      np.std(run_fitness),
        "Mean PSO Time":    np.mean(run_times),
        "Median PSO Time":  np.median(run_times),
        "Std PSO Time":     np.std(run_times),
    }


# --- Print summary ------------------------------------------------------------
metrics = [
    "Max Fitness",
    "Min Fitness",
    "Mean Fitness",
    "Median Fitness",
    "Std Fitness",
    "Mean PSO Time",
    "Median PSO Time",
    "Std PSO Time",
]

print(f"\n\n{'='*60}\nPSO PERFORMANCE SUMMARY\n{'='*60}")
print(f"{'Metric':<25}" + "".join(f"{n:>10}" for n in variants))

for metric in metrics:
    print(f"{metric:<25}" + "".join(
        f" {all_results[n][metric]:>10.4f}" for n in variants
    ))
