# ── Grid ──────────────────────────────────────────
NX = 10
NY = 10
GRID_SIZE = NX*NY

# ── Robots ────────────────────────────────────────
NUM_ROBOTS = 3
OBS_POS = [5,6,7] #obstacle position
PORTIONS = [1/3, 1/3, 1/3] # porition per robot
NOT_EQUAL = True # set True if using unequal portions above

# ── DARP ──────────────────────────────────────────
DARP_PSO_ITER = 2000  #for PSO
DARP_FINAL_ITER = 80000 # final DARP

## ── Panel Size and Robot velocity ─────────────────────────────────────
CELL_WIDTH = 1.1 # Assume the brush covers this much width (metres)
SPEED_ROBOT = 0.25 # Assume the velocity of the robot (metres/second)

# ── Traversal ─────────────────────────────────────
CELL_TIME = CELL_WIDTH/SPEED_ROBOT # (seconds)
TURN_TIME = 2 # Assume the turn time of the robot for 90 degrees (seconds)

# ── PSO ───────────────────────────────────────────
NUM_PARTICLES = 10
MAXITER = 15

# ── Penalties ─────────────────────────────────────
FAILURE_PENALTY = 1000.0