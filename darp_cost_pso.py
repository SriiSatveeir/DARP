from multiRobotPathPlanner import MultiRobotPathPlanner
from pso_parameter import NX, NY, OBS_POS, NUM_ROBOTS, PORTIONS, NOT_EQUAL, DARP_PSO_ITER, DARP_FINAL_ITER, CELL_TIME, TURN_TIME, FAILURE_PENALTY


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
OBS_SET    = set(OBS_POS)
OUTER_CELLS = [c for c in range(NX * NY)
               if (c // NY in (0, NX-1) or c % NY in (0, NY-1))
               and c not in OBS_SET]

def get_outer_cells():
    return OUTER_CELLS

def validate_config(outer_cells):
    """
    Check that the problem is solvable before running PSO.
    Exits early with a clear message if not.
    """
    obs_on_border = [c for c in OBS_POS
                     if c // NY in (0, NX-1) or c % NY in (0, NY-1)]

    available = len(outer_cells)   # obstacles already excluded

    print(f"Grid                : {NX} x {NY}")
    print(f"Outer cells total   : {2*NX + 2*NY - 4}")
    print(f"Obstacles on border : {len(obs_on_border)}  {obs_on_border}")
    print(f"Available positions : {available}")
    print(f"Robots needed       : {NUM_ROBOTS}")

    if available < NUM_ROBOTS:
        raise ValueError(
            f"Not enough valid outer cells ({available}) "
            f"for {NUM_ROBOTS} robots. "
            f"Reduce NUM_ROBOTS or remove border obstacles."
        )

    # warn if it's tight — leaves very little diversity for PSO
    if available < NUM_ROBOTS * 3:
        print(f"WARNING: only {available} positions for {NUM_ROBOTS} robots "
              f"— PSO diversity will be limited.")

def darp_cost(positions):
    """
    Fitness function passed to PSO minimize().

    Parameters
    ----------
    positions : list of int
        Outer-cell indices e.g. [3, 27, 41]
        Converted to real grid cells via outer_cells[p] before passing to DARP.

    Returns
    -------
    float : mission_time — time for all robots to finish (slowest robot).
            Lower is better. Returns FAILURE_PENALTY if DARP cannot solve.
    """
    

    real_positions = [OUTER_CELLS[int(p)] for p in positions]

    if len(set(real_positions)) != len(real_positions):
        return FAILURE_PENALTY
    if any(p in OBS_SET for p in real_positions):
        return FAILURE_PENALTY
    
    # # constraint checks before running DARP (fast)
    # if len(set(positions)) != len(positions):   # duplicate positions
    #     return FAILURE_PENALTY
    # if any(p in OBS_POS for p in positions):    # position on obstacle
    #     return FAILURE_PENALTY
    # if any(p < 0 or p >= NX * NY for p in positions):  # outside grid
    #     return FAILURE_PENALTY

    try:
        planner = MultiRobotPathPlanner(
            nx               = NX,
            ny               = NY,
            notEqualPortions = NOT_EQUAL,
            initial_positions= real_positions,
            portions         = PORTIONS,
            obs_pos          = OBS_POS,
            visualization    = False,       # never visualise during PSO
            MaxIter          = DARP_PSO_ITER,
            cell_time        = CELL_TIME,
            turn_time        = TURN_TIME,
            verbose          = False
        )

        if not planner.DARP_success:
            return FAILURE_PENALTY

        return planner.mission_time         # this is the fitness value PSO minimises

    except SystemExit:
        # DARP calls sys.exit() on invalid configs — treat as failure
        return FAILURE_PENALTY
    except Exception:
        return FAILURE_PENALTY

#--- multirobotpathplanner final run ----------------------------------------------------------------------+

def final_run(best_positions, visualize=True):

    print("\n" + "="*60)
    print("FINAL HIGH-QUALITY RUN WITH BEST POSITIONS")
    print("="*60)

    real_positions = [OUTER_CELLS[int(p)] for p in best_positions]
    
    planner = MultiRobotPathPlanner(
        nx=NX, ny=NY,
        notEqualPortions=NOT_EQUAL,
        initial_positions=real_positions,
        portions=PORTIONS,
        obs_pos=OBS_POS,
        visualization=visualize,
        MaxIter=DARP_FINAL_ITER,
        cell_time=CELL_TIME,
        turn_time=TURN_TIME
    )
    return planner

#--- END ----------------------------------------------------------------------+
