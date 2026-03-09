# ── Grid ──────────────────────────────────────────
NX = 10
NY = 10
GRID_SIZE = NX*NY

# ── Robots ────────────────────────────────────────
NUM_ROBOTS = 3
OBS_POS = [5,6,7]
PORTIONS = [1/3, 1/3, 1/3]
NOT_EQUAL = True # set True if using unequal portions above

# ── DARP ──────────────────────────────────────────
DARP_MAX_ITER = 2000  #for PSO
DARP_FINAL_ITER = 80000

# ── Traversal ─────────────────────────────────────
CELL_TIME = 1.0
TURN_PENALTY = 0.5

# ── PSO ───────────────────────────────────────────
NUM_PARTICLES = 10
MAXITER = 15

# ── Penalties ─────────────────────────────────────
FAILURE_PENALTY = 1000.0