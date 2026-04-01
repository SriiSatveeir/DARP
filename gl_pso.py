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

def sample_valid_positions(num_robots, grid_size, obs_pos):
    # pick num_robots unique cell indices that are not obstacles
    valid_cells = [c for c in range(grid_size) if c not in obs_pos]
    return rnd.sample(valid_cells, num_robots)   # rnd.sample = no duplicates

# def sample_valid_positions(num_robots, grid_size, obs_pos):
#     outer = []
#     for c in range(grid_size):
#         row = c // NY
#         col = c % NY
#         if row == 0 or row == NX-1 or col == 0 or col == NY-1:
#             if c not in obs_pos:
#                 outer.append(c)
#     # pick num_robots unique cell indices that are not obstacles
#     return rnd.sample(outer, num_robots)

def resolve_conflicts(positions, obs_pos, grid_size):
    # after rounding, two robots may share a cell or land on obstacle
    # nudge any conflicting robot to nearest free valid cell
    obs_set = set(obs_pos)
    taken   = set()
    result  = []

    for p in positions:
        if p not in taken and p not in obs_set and 0 <= p < grid_size:
            result.append(p)
            taken.add(p)
        else:
            found = False
            for offset in range(1, grid_size):
                for candidate in [p + offset, p - offset]:
                    if (0 <= candidate < grid_size
                            and candidate not in taken
                            and candidate not in obs_set):
                        result.append(candidate)
                        taken.add(candidate)
                        found = True
                        break
                if found:
                    break
            if not found:
                result.append(p)  # fallback

    return result


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

        if not self.pos_best_i:
            return
        
        for i in range(0, num_dimensions):
            r = random()

            if self.pos_best_i[i] != 0 and pos_best_g[i] !=0:
                ratio = pos_best_g[i] / self.pos_best_i[i]
            else:
                ratio = 1.0

            c = 1.0 + ratio
            w = 1.1 - ratio
            
            vel_combined = c * r * (self.pos_best_i[i] + pos_best_g[i] - 2 * self.position_i[i])
            self.velocity_i[i] = w * self.velocity_i[i] + vel_combined

    # update the particle position based off new velocity updates
    def update_position(self, bounds, obs_pos, grid_size):  # CHANGED: added obs_pos, grid_size
        for i in range(0, num_dimensions):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

            if self.position_i[i] > bounds[i][1]:
                self.position_i[i] = bounds[i][1]
            if self.position_i[i] < bounds[i][0]:
                self.position_i[i] = bounds[i][0]

            self.position_i[i] = int(round(self.position_i[i]))  # CHANGED: round to integer cell index

        self.position_i = resolve_conflicts(self.position_i, obs_pos, grid_size)  # CHANGED: fix conflicts


def minimize(costFunc, bounds, num_robots, grid_size, obs_pos,  # CHANGED: added num_robots, grid_size, obs_pos
             num_particles, maxiter, verbose=False):
    start = time.time()
    global num_dimensions
    num_dimensions = num_robots     # CHANGED: one dimension = one robot starting position

    err_best_g = -1                 # best error for group
    pos_best_g = []                 # best position for group

    # establish the swarm
    swarm = []
    for i in range(0, num_particles):
        x0 = sample_valid_positions(num_robots, grid_size, obs_pos)  # CHANGED: each particle gets its own random valid start
        swarm.append(Particle(x0))

    # begin optimization loop
    i = 0
    while i < maxiter:
    
        # cycle through particles in swarm and evaluate fitness
        for j in range(0, num_particles):
            swarm[j].evaluate(costFunc)

            if swarm[j].err_i < err_best_g or err_best_g == -1:
                pos_best_g = list(swarm[j].position_i)
                err_best_g = float(swarm[j].err_i)

        if verbose: print(f'iter: {i+1:>4d}, best solution: {err_best_g:10.6f}')

        # cycle through swarm and update velocities and position
        for j in range(0, num_particles):
            swarm[j].update_velocity(pos_best_g)
            swarm[j].update_position(bounds, obs_pos, grid_size)  # CHANGED: pass obs_pos, grid_size

        i += 1

    # print final results
    if verbose:
        print('\nFINAL SOLUTION:')
        print(f'   > {pos_best_g}')
        print(f'   > {err_best_g}\n')

    pso_time = time.time() - start
    return err_best_g, pos_best_g, pso_time

#--- END ----------------------------------------------------------------------+


#--- RUN ----------------------------------------------------------------------+

if __name__ == "__main__":

    # environment settings — edit these to match your solar panel grid
    
    # outer =  [c for c in range(GRID_SIZE) 
    #      if c // NY == 0 or c // NY == NX-1 
    #      or c % NY == 0 or c % NY == NY-1
    #      and c not in OBS_POS]
    # bounds = [(min(outer), max(outer))] * NUM_ROBOTS

    bounds = [(0, GRID_SIZE - 1)] * NUM_ROBOTS

    best_fitness, best_positions, pso_time = minimize(
        costFunc      = darp_cost,
        bounds        = bounds,
        num_robots    = NUM_ROBOTS,
        grid_size     = GRID_SIZE,
        obs_pos       = OBS_POS,
        num_particles = NUM_PARTICLES,
        maxiter       = MAXITER,
        verbose       = True
    )

    print(f"\nBest robot starting positions : {best_positions}")
    print(f"Best mission time (fitness)   : {best_fitness:.4f}")
    print(f"PSO search time               : {pso_time:.3f} seconds")

    final_run(best_positions, visualize=True)  