# ── Grid ──────────────────────────────────────────
NX = 25
NY = 25
GRID_SIZE = NX*NY

# ── Robots ────────────────────────────────────────
NUM_ROBOTS = 3
OBS_POS = [
    # --- Vertical wall left (col=5) ---
    3*NY + 5, 4*NY + 5, 5*NY + 5,
    9*NY + 5, 10*NY + 5, 11*NY + 5,

    # --- Vertical wall right (col=19) ---   ← mirror of col=5
    3*NY + 19, 4*NY + 19, 5*NY + 19,
    9*NY + 19, 10*NY + 19, 11*NY + 19,

    # --- Horizontal wall top (row=4) ---
    4*NY + 2, 4*NY + 3, 4*NY + 4,           # left of left wall
    4*NY + 6, 4*NY + 7, 4*NY + 8,
    4*NY + 17, 4*NY + 18,           # between walls
    4*NY + 20, 4*NY + 21, 4*NY + 22,        # right of right wall ← mirror

    # --- Horizontal wall bottom (row=10) ---
    10*NY + 2, 10*NY + 3, 10*NY + 4,
    10*NY + 6, 10*NY + 7, 10*NY + 8,
    10*NY + 17, 10*NY + 18,
    10*NY + 20, 10*NY + 21, 10*NY + 22,
]
# OBS_POS = [
#     # --- Vertical wall (left section) ---
#     2*NY + 5, 3*NY + 5, 4*NY + 5, 5*NY + 5, 6*NY + 5,
#     8*NY + 5, 9*NY + 5, 10*NY + 5, 11*NY + 5, 12*NY + 5,

#     # --- Vertical wall (right section) ---
#     2*NY + 10, 3*NY + 10, 4*NY + 10, 5*NY + 10, 6*NY + 10,
#     8*NY + 10, 9*NY + 10, 10*NY + 10, 11*NY + 10, 12*NY + 10,

#     # --- Horizontal wall (top section) ---
#     6*NY + 2, 6*NY + 3, 6*NY + 4,
#     6*NY + 6, 6*NY + 7, 6*NY + 8,
#     6*NY + 11, 6*NY + 12,

#     # --- Horizontal wall (bottom section) ---
#     10*NY + 2, 10*NY + 3, 10*NY + 4,
#     10*NY + 6, 10*NY + 7, 10*NY + 8,
#     10*NY + 11, 10*NY + 12,
# ]#obstacle position for first test or old obs
PORTIONS = [1/3, 1/3, 1/3] # porition per robot
NOT_EQUAL = False # set True if using unequal portions above

# ── DARP ──────────────────────────────────────────
DARP_PSO_ITER = 20  #for PSO
DARP_FINAL_ITER = 80000 # final DARP

## ── Panel Size and Robot velocity ─────────────────────────────────────
CELL_WIDTH = 1.1 # Assume the brush covers this much width (metres)
SPEED_ROBOT = 0.25 # Assume the velocity of the robot (metres/second)

# ── Traversal ─────────────────────────────────────
CELL_TIME = CELL_WIDTH/SPEED_ROBOT # (seconds)
TURN_TIME = 2 # Assume the turn time of the robot for 90 degrees (seconds)

# ── PSO ───────────────────────────────────────────
NUM_PARTICLES = 12 #12
MAXITER = 25 #30

# ── Penalties ─────────────────────────────────────
FAILURE_PENALTY = NX * NY * CELL_TIME * 10