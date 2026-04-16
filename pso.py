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

def build_outer_coords(outer_cells): # Outer cell indices to (row,col) format
    coords = []
    for cell in outer_cells:
        row = cell // NY 
        col = cell % NY
        coords.append((row,col))
    return np.array(coords)

def snap_to_outer_cells(robot_positions, outer_coords, outer_cells): #assign outer cells to the robot
    taken = set() #Track already set outer cells
    result = []

    for pos in robot_positions:
        row, col = pos
        distances = np.sqrt( (outer_coords[:, 0] - row) ** 2 + (outer_coords[:, 1] - col) ** 2) #distance from estimated position to the outer cell

        for idx in np.argsort(distances): #sort the outer cells by closest distance
                idx = int(idx)
                if idx not in taken: #assign the closest outer cell not taken
                    result.append(idx)
                    taken.add(idx)
                    break
    return result

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
    def update_velocity(self, pos_best_g):
        w  = 0.5    # constant inertia weight
        c1 = 1      # cognitive constant
        c2 = 2      # social constant

        for i in range(len(self.position_i)):
            r1 = random()
            r2 = random()

            vel_cognitive = c1 * r1 * (self.pos_best_i[i] - self.position_i[i])
            vel_social    = c2 * r2 * (pos_best_g[i]      - self.position_i[i])
            self.velocity_i[i] = w * self.velocity_i[i] + vel_cognitive + vel_social

            is_row_dim = (i % 2 == 0)
            v_max = NX * 0.2 if is_row_dim else NY * 0.2
            self.velocity_i[i] = max(-v_max, min(v_max, self.velocity_i[i]))

    # update the particle position based off new velocity updates
    def update_position(self):

        for i in range(len(self.position_i)):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

            is_row_dim = (i % 2 == 0)
            max_val = NX - 1.0 if is_row_dim else NY - 1.0
            self.position_i[i] = max(0.0, min(max_val, self.position_i[i]))

            # if self.position_i[i] > len(outer_cells)-1:
            #     self.position_i[i] = len(outer_cells)-1
            # if self.position_i[i] < 0:
            #     self.position_i[i] = 0
        #     self.position_i[i] = int(round(self.position_i[i]))
        # self.position_i = resolve_conflicts(self.position_i, outer_cells)

            # if self.position_i[i] > bounds[i][1]:
            #     self.position_i[i] = bounds[i][1]
            # if self.position_i[i] < bounds[i][0]:
            #     self.position_i[i] = bounds[i][0]

            # self.position_i[i] = int(round(self.position_i[i]))  # CHANGED: round to integer cell index

        # self.position_i = resolve_conflicts(self.position_i, obs_pos, grid_size)  # CHANGED: fix conflicts


def minimize(costFunc, verbose=False):
    start = time.time()

    outer_cells  = get_outer_cells()   # border cells excluding obstacles
    outer_coords = build_outer_coords(outer_cells)

    err_best_g = float('inf')                 # best error for group
    pos_best_g = []            # continuous (row, col)
    pos_best_g_discrete = []   # outer cell indices → passed to DARP
    
    history = []

    # establish the swarm
    swarm = []
    for j in range(0, NUM_PARTICLES):
        swarm.append(Particle(outer_cells, outer_coords))

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

        # cycle through swarm and update velocities and position
        for j in range(NUM_PARTICLES):
            swarm[j].update_velocity(pos_best_g)
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

    best_fitness, best_positions, pso_time, history, converge_iter = minimize( costFunc = darp_cost, verbose = True) #return err_best_g, pos_best_g, pso_time

    outer_cells = get_outer_cells()
    outer_coords = build_outer_coords(outer_cells)

    real_positions = [outer_cells[int(p)] for p in best_positions]
    real_positions_brac = [(p // NY, p % NY) for p in real_positions]

    print(f"\nBest robot starting positions : {real_positions_brac}")
    print(f"Best mission time (fitness)   : {best_fitness:.4f}")
    print(f"PSO search time               : {pso_time:.3f} seconds")
    print(f"Iterations to converge        : {converge_iter}")

    final_run(best_positions, visualize=True)