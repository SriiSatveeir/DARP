# from darp import DARP
# import numpy as np

# def fitness_function(positions):

#     darp_solver = DARP(
#         nx=20,
#         ny=20,
#         notEqualPortions=False,
#         given_initial_positions=positions.astype(int),
#         given_portions=[],
#         obstacles_positions=[],
#         visualization=False
#     )

#     success, _ = darp_solver.divideRegions()

#     if not success:
#         return 1e9   # penalty

#     loads = darp_solver.ArrayOfElements
#     imbalance = np.std(loads)

#     return imbalance

#------------------------------------------------------------------------------+
#
#	Nathan A. Rooy
#	Simple Particle Swarm Optimization (PSO) with Python
#	Last update: 2018-JAN-26
#	Python 3.6
#
#------------------------------------------------------------------------------+

#--- IMPORT DEPENDENCIES ------------------------------------------------------+

from random import random
from random import uniform
import random as rnd
from pso_parameter import NUM_ROBOTS, NUM_PARTICLES, MAXITER, NX, NY, OBS_POS
from darp_cost_pso import darp_cost, final_run, get_outer_cells
import time
import numpy as np

#--- HELPERS (new, not in Nathan's original) ----------------------------------+
def build_outer_coords(outer_cells): #Convert the outer cell indices to (row,col) format
    coords = []
    for cell in outer_cells:
        row = cell // NY
        col = cell % NY
        coords.append((row,col))
    return np.array(coords)

def snap_to_outer_cells(robot_positions, outer_coords, outer_cells): #assign outer cells to the robot
    taken = set()
    result = []

    for pos in robot_positions:
        row, col = pos
        distances = np.sqrt( (outer_coords[:, 0] - row) ** 2 + (outer_coords[:, 1] - col) ** 2) #distance from estimated position to cell

        for idx in np.argsort(distances): #sort in ascending order
                idx = int(idx)
                if idx not in taken:
                    result.append(idx)
                    taken.add(idx)
                    break
    return result

        
    
# def sample_valid_positions(num_robots, grid_size, obs_pos):
#     # pick num_robots unique cell indices that are not obstacles
#     valid_cells = [c for c in range(grid_size) if c not in obs_pos]
#     return rnd.sample(valid_cells, num_robots)   # rnd.sample = no duplicates

# def get_outer_cells():
#     obs_set = set(OBS_POS)
#     return [c for c in range (NX*NY)
#             if (c // NY in (0, NX-1) or c % NY in (0, NY-1)) and c not in obs_set]

# def sample_valid_positions(outer_cells):
#     return rnd.sample(range(len(outer_cells)), NUM_ROBOTS)

# def resolve_conflicts(position, outer_cells):
#     taken = set()
#     result = []
#     available = set(range(len(outer_cells)))

#     for p in position:
#         p = max(0, min(len(outer_cells)-1, p))

#         if p not in taken:
#             result.append(p)
#             taken.add(p)
#         else:
#             choices = list(available - taken)
#             if choices:
#                 new_p = rnd.choice(choices)
#                 result.append(new_p)
#                 taken.add(new_p)
#             else:
#                 result.append(p)
#                 taken.add(p)

#     return result

#--- OLD CODE ---------------------------------------------------------------------+
# def sample_valid_positions(num_robots, grid_size, obs_pos):
#     outer = []
#     for c in range(grid_size):
#         row = c // NY
#         col = c % NY
#         if row == 0 or row == NX-1 or col == 0 or col == NY-1:
#             if c not in obs_pos:
#                 outer.append(c)
#     # pick num_robots unique cell indices that are not obstacles
#     return rnd.sample(outer, num_robots)   # rnd.sample = no duplicates


# def resolve_conflicts(positions, obs_pos, grid_size):
#     # after rounding, two robots may share a cell or land on obstacle
#     # nudge any conflicting robot to nearest free valid cell
#     obs_set = set(obs_pos)
#     taken   = set()
#     result  = []

#     for p in positions:
#         if p not in taken and p not in obs_set and 0 <= p < grid_size:
#             result.append(p)
#             taken.add(p)
#         else:
#             found = False
#             for offset in range(1, grid_size):
#                 for candidate in [p + offset, p - offset]:
#                     if (0 <= candidate < grid_size
#                             and candidate not in taken
#                             and candidate not in obs_set):
#                         result.append(candidate)
#                         taken.add(candidate)
#                         found = True
#                         break
#                 if found:
#                     break
#             if not found:
#                 result.append(p)  # fallback

#     return result


#--- MAIN ---------------------------------------------------------------------+

class Particle:
    def __init__(self, outer_cells, outer_coords):
        n = len(outer_cells)

        chosen_indices = rnd.sample(range(n), NUM_ROBOTS)
        self.position_i = []        # particle position
        self.velocity_i = []        # particle velocity

        for idx in chosen_indices:
            row, col = outer_coords[idx]

            # position: starts at a known valid outer cell
            self.position_i.append(float(row))
            self.position_i.append(float(col))

            # velocity: small random start
            self.velocity_i.append(rnd.uniform(-NX * 0.2, NX * 0.2))  # row
            self.velocity_i.append(rnd.uniform(-NY * 0.2, NY * 0.2))  # col

        # personal best = starting position (safe fallback before first eval)
        self.pos_best_i = self.position_i.copy()
        self.pos_best_discrete = snap_to_outer_cells(self._to_pairs(self.position_i), outer_coords, outer_cells)

        self.err_best_i = float('inf')        # best error individual
        self.err_i      = float('inf')        # error individual
        
    def _to_pairs(self, flat_list): #Convert flat [r0, c0, r1, c1, ...] into [[r0,c0], [r1,c1], ...]
        pairs = []
        for i in range(0, len(flat_list), 2):
            pairs.append([flat_list[i], flat_list[i + 1]])
        return pairs

    # evaluate current fitness
    def evaluate(self, costFunc, outer_cells, outer_coords):
        robot_pairs  = self._to_pairs(self.position_i)
        discrete_pos = snap_to_outer_cells(robot_pairs, outer_coords, outer_cells)
        self.err_i   = costFunc(discrete_pos)

        if self.err_i < self.err_best_i:
            self.pos_best_i = self.position_i.copy()
            self.pos_best_discrete = discrete_pos.copy()
            self.err_best_i = self.err_i

    # update new particle velocity
    def update_velocity(self, pos_best_g, w0):
        c1 = 2.0      # cognitive constant
        c2 = 2.0      # social constant

        for i in range(len(self.position_i)):
            r1=random()
            r2=random()
            
            vel_cognitive = c1 * r1 * (self.pos_best_i[i] - self.position_i[i])
            vel_social    = c2 * r2 * (pos_best_g[i]      - self.position_i[i])
            self.velocity_i[i] = w0 * self.velocity_i[i] + vel_cognitive + vel_social

            is_row_dim = (i % 2 == 0)
            v_max = NX * 0.2 if is_row_dim else NY * 0.2
            self.velocity_i[i] = max(-v_max, min(v_max, self.velocity_i[i]))

    # update the particle position based off new velocity updates
    def update_position(self):  # CHANGED: added obs_pos, grid_size
        for i in range(len(self.position_i)):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

            is_row_dim = (i % 2 == 0)
            max_val = NX - 1.0 if is_row_dim else NY - 1.0
            self.position_i[i] = max(0.0, min(max_val, self.position_i[i]))
        #     self.position_i[i] = int(round(self.position_i[i]))
        # self.position_i = resolve_conflicts(self.position_i, outer_cells)

            # if self.position_i[i] > bounds[i][1]:
            #     self.position_i[i] = bounds[i][1]
            # if self.position_i[i] < bounds[i][0]:
            #     self.position_i[i] = bounds[i][0]

            # self.position_i[i] = int(round(self.position_i[i]))  # CHANGED: round to integer cell index

        # self.position_i = resolve_conflicts(self.position_i, obs_pos, grid_size)  # CHANGED: fix conflicts


def minimize(costFunc, w=0.9, u=1.0005, verbose=False):
    start = time.time()

    outer_cells  = get_outer_cells()   # border cells excluding obstacles
    outer_coords = build_outer_coords(outer_cells)

    err_best_g = float('inf')                 # best error for group
    pos_best_g = []            # continuous (row, col)
    pos_best_g_discrete = []   # outer cell indices → passed to DARP

    history = []

    # establish the swarm
    swarm = []
    for i in range(0, NUM_PARTICLES):
        swarm.append(Particle(outer_cells, outer_coords))

    # intial particle positions
    # print("\nInitial particle positions (outer cell indices):")
    # for i, particle in enumerate(swarm):
    #     real_positions = [outer_cells[int(p)] for p in particle.position_i]
    #     real_positions_brac = [(p // NY, p % NY) for p in real_positions]
    #     print(f"  Particle {i+1}: {real_positions_brac}")
    # print()

    # begin optimization loop
    i = 0
    while i < MAXITER:
    
        # cycle through particles in swarm and evaluate fitness
        for j in range(NUM_PARTICLES):
            swarm[j].evaluate(costFunc, outer_cells, outer_coords)

            if swarm[j].err_i < err_best_g:
                err_best_g          = float(swarm[j].err_i)
                pos_best_g          = swarm[j].pos_best_i.copy()
                pos_best_g_discrete = swarm[j].pos_best_discrete.copy()

        if verbose:
            print(f'iter: {i+1:>4d}, best solution: {err_best_g:10.6f}')

        w0 = w * (u ** (-i))
        w0 = min(1.0, max(0.0, w0))

        # cycle through swarm and update velocities and position
        for j in range(NUM_PARTICLES):
            swarm[j].update_velocity(pos_best_g, w0)
            swarm[j].update_position()  # CHANGED: pass obs_pos, grid_size

        i += 1
        history.append(err_best_g)

    converge_iter = None
    for idx, val in enumerate(history):
        if val == err_best_g:
            converge_iter = idx + 1
            break

    # print final results
    if verbose:
        print('\nFINAL SOLUTION:')
        print(f'   > best position: {pos_best_g}')
        print(f'   >best fitness : {err_best_g}\n')

    pso_time = time.time() - start
    return err_best_g, pos_best_g_discrete, pso_time, history, converge_iter

#--- END ----------------------------------------------------------------------+


#--- RUN ----------------------------------------------------------------------+

if __name__ == "__main__":
    # environment settings — edit these to match your solar panel grid
    
    # outer =  [c for c in range(GRID_SIZE) 
    #      if c // NY == 0 or c // NY == NX-1 
    #      or c % NY == 0 or c % NY == NY-1
    #      and c not in OBS_POS]
    # bounds = [(min(outer), max(outer))] * NUM_ROBOTS

    # bounds = [(0, GRID_SIZE - 1)] * NUM_ROBOTS

    best_fitness, best_positions, pso_time, history, converge_iter = minimize( costFunc = darp_cost, verbose = True) #return err_best_g, pos_best_g, pso_time

    outer_cells = get_outer_cells()
    real_positions = [outer_cells[int(p)] for p in best_positions]
    real_positions_brac = [(p // NY, p % NY) for p in real_positions]

    print(f"\nBest robot starting positions : {real_positions_brac}")
    print(f"Best mission time (fitness)   : {best_fitness:.4f}")
    print(f"PSO search time               : {pso_time:.3f} seconds")
    print(f"Iterations to converge        : {converge_iter}")

    final_run(best_positions, visualize=True)