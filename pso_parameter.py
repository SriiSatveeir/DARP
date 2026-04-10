# ── Grid ──────────────────────────────────────────
NX = 15
NY = 15
GRID_SIZE = NX*NY

# ── Robots ────────────────────────────────────────
NUM_ROBOTS = 3
OBS_POS = [
    7*NY + 3, 7*NY + 4, 7*NY + 5,
    10*NY + 10, 11*NY + 10, 12*NY + 10,
    3*NY + 7, 4*NY + 7, 5*NY + 7
] #obstacle position
PORTIONS = [1/3, 1/3, 1/3] # porition per robot
NOT_EQUAL = False # set True if using unequal portions above

# ── DARP ──────────────────────────────────────────
DARP_PSO_ITER = 500  #for PSO
DARP_FINAL_ITER = 80000 # final DARP

## ── Panel Size and Robot velocity ─────────────────────────────────────
CELL_WIDTH = 1.1 # Assume the brush covers this much width (metres)
SPEED_ROBOT = 0.25 # Assume the velocity of the robot (metres/second)

# ── Traversal ─────────────────────────────────────
CELL_TIME = CELL_WIDTH/SPEED_ROBOT # (seconds)
TURN_TIME = 2 # Assume the turn time of the robot for 90 degrees (seconds)

# ── PSO ───────────────────────────────────────────
NUM_PARTICLES = 10
MAXITER = 20

# ── Penalties ─────────────────────────────────────
FAILURE_PENALTY = NX * NY * CELL_TIME * 10