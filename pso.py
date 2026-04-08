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
from pso_parameter import *
from darp_cost_pso import darp_cost, final_run
import time

#--- HELPERS (new, not in Nathan's original) ----------------------------------+

# def sample_valid_positions(num_robots, grid_size, obs_pos):
#     # pick num_robots unique cell indices that are not obstacles
#     valid_cells = [c for c in range(grid_size) if c not in obs_pos]
#     return rnd.sample(valid_cells, num_robots)   # rnd.sample = no duplicates

def get_outer_cells():
    obs_set = set(OBS_POS)
    return [c for c in range (NX*NY)
            if (c // NY in (0, NX-1) or c % NY in (0, NY-1)) and c not in obs_set]

def sample_valid_positions(outer_cells):
    return rnd.sample(range(len(outer_cells)), NUM_ROBOTS)

def resolve_conflicts(position, outer_cells):
    taken = set()
    result = []
    for p in position:
        p = int(round(p))
        p = max(0, min(len(outer_cells)-1, p))  # clamp to valid index range
        while p in taken:
            p = (p + 1) % len(outer_cells)  # nudge to next free index
        result.append(p)
        taken.add(p)
    return result

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
    def __init__(self, x0):
        self.position_i = []        # particle position
        self.velocity_i = []        # particle velocity
        self.pos_best_i = []        # best position individual
        self.err_best_i = -1        # best error individual
        self.err_i      = -1        # error individual

        for i in range(0, num_dimensions):
            self.velocity_i.append(uniform(-1, 1))
            self.position_i.append(x0[i])

    # evaluate current fitness
    def evaluate(self, costFunc):
        self.err_i = costFunc(self.position_i)

        if self.err_i < self.err_best_i or self.err_best_i == -1:
            self.pos_best_i = self.position_i.copy()
            self.err_best_i = self.err_i

    # update new particle velocity
    def update_velocity(self, pos_best_g):
        w  = 0.5    # constant inertia weight
        c1 = 1      # cognitive constant
        c2 = 2      # social constant

        for i in range(0, num_dimensions):
            r1 = random()
            r2 = random()

            vel_cognitive = c1 * r1 * (self.pos_best_i[i] - self.position_i[i])
            vel_social    = c2 * r2 * (pos_best_g[i]      - self.position_i[i])
            self.velocity_i[i] = w * self.velocity_i[i] + vel_cognitive + vel_social

    # update the particle position based off new velocity updates
    def update_position(self, outer_cells):  # CHANGED: added obs_pos, grid_size
        for i in range(0, num_dimensions):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

            if self.position_i[i] > len(outer_cells)-1:
                self.position_i[i] = len(outer_cells)-1
            if self.position_i[i] < 0:
             self.position_i[i] = 0
            self.position_i[i] = int(round(self.position_i[i]))
        self.position_i = resolve_conflicts(self.position_i, outer_cells)

            # if self.position_i[i] > bounds[i][1]:
            #     self.position_i[i] = bounds[i][1]
            # if self.position_i[i] < bounds[i][0]:
            #     self.position_i[i] = bounds[i][0]

            # self.position_i[i] = int(round(self.position_i[i]))  # CHANGED: round to integer cell index

        # self.position_i = resolve_conflicts(self.position_i, obs_pos, grid_size)  # CHANGED: fix conflicts


def minimize(costFunc, verbose=False):
    start = time.time()
    global num_dimensions
    num_dimensions = NUM_ROBOTS     # CHANGED: one dimension = one robot starting position

    err_best_g = -1                 # best error for group
    pos_best_g = []                 # best position for group
    
    outer_cells = get_outer_cells()

    history = []

    # establish the swarm
    swarm = []
    for i in range(0, NUM_PARTICLES):
        x0 = sample_valid_positions(outer_cells)  # CHANGED: each particle gets its own random valid start
        swarm.append(Particle(x0))

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
        for j in range(0, NUM_PARTICLES):
            swarm[j].evaluate(costFunc)

            if swarm[j].err_i < err_best_g or err_best_g == -1:
                pos_best_g = list(swarm[j].position_i)
                err_best_g = float(swarm[j].err_i)

        if verbose: print(f'iter: {i+1:>4d}, best solution: {err_best_g:10.6f}')

        # cycle through swarm and update velocities and position
        for j in range(0, NUM_PARTICLES):
            swarm[j].update_velocity(pos_best_g)
            swarm[j].update_position(outer_cells)  # CHANGED: pass obs_pos, grid_size

        i += 1
        history.append(err_best_g)

    converge_iter = history.index(err_best_g) + 1

    # print final results
    if verbose:
        print('\nFINAL SOLUTION:')
        print(f'   > {pos_best_g}')
        print(f'   > {err_best_g}\n')

    pso_time = time.time() - start
    return err_best_g, pos_best_g, pso_time, history, converge_iter

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