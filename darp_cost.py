from multiRobotPathPlanner import MultiRobotPathPlanner
from pso_parameter import *

# --- environment settings (must match what you set in pso.py) ----------------

# NX             = 10
# NY             = 10
# OBS_POS        = [5, 6, 7]
# NUM_ROBOTS     = 3
# PORTIONS       = [1/3, 1/3, 1/3]   # equal split — change if robots cover different areas
# NOT_EQUAL      = False              # set True if using unequal portions above

# DARP_MAX_ITER  = 2000               # low during PSO for speed; final run uses 80000
# CELL_TIME      = 1.0                # time units per cell traversed
# TURN_PENALTY   = 0.5                # extra time units per turn

# FAILURE_PENALTY = 1000.0            # returned if DARP fails or positions are invalid

# # -----------------------------------------------------------------------------


def darp_cost(positions):
    """
    Fitness function passed to PSO minimize().

    Parameters
    ----------
    positions : list of int
        Robot starting cell indices e.g. [3, 45, 82]
        Robot 1 starts at cell 3, Robot 2 at cell 45, Robot 3 at cell 82.

    Returns
    -------
    float : mission_time — time for all robots to finish (slowest robot).
            Lower is better. Returns FAILURE_PENALTY if DARP cannot solve.
    """

    # constraint checks before running DARP (fast)
    if len(set(positions)) != len(positions):   # duplicate positions
        return FAILURE_PENALTY
    if any(p in OBS_POS for p in positions):    # position on obstacle
        return FAILURE_PENALTY
    if any(p < 0 or p >= NX * NY for p in positions):  # outside grid
        return FAILURE_PENALTY

    try:
        planner = MultiRobotPathPlanner(
            nx               = NX,
            ny               = NY,
            notEqualPortions = NOT_EQUAL,
            initial_positions= [int(p) for p in positions],
            portions         = PORTIONS,
            obs_pos          = OBS_POS,
            visualization    = False,       # never visualise during PSO
            MaxIter          = DARP_MAX_ITER,
            cell_time        = CELL_TIME,
            turn_penalty     = TURN_PENALTY
        )

        if not planner.DARP_success:
            return FAILURE_PENALTY

        return planner.mission_time         # this is the fitness value PSO minimises

    except SystemExit:
        # DARP calls sys.exit() on invalid configs — treat as failure
        return FAILURE_PENALTY
    except Exception:
        return FAILURE_PENALTY

#--- multirobotpathplanner ----------------------------------------------------------------------+

def final_run(best_positions, visualize=True):
    print("\n" + "="*60)
    print("FINAL HIGH-QUALITY RUN WITH BEST POSITIONS")
    print("="*60)
    planner = MultiRobotPathPlanner(
        nx=NX, ny=NY,
        notEqualPortions=NOT_EQUAL,
        initial_positions=[int(p) for p in best_positions],
        portions=PORTIONS,
        obs_pos=OBS_POS,
        visualization=visualize,
        MaxIter=DARP_FINAL_ITER,
        cell_time=CELL_TIME,
        turn_penalty=TURN_PENALTY
    )
    return planner

#--- END ----------------------------------------------------------------------+
