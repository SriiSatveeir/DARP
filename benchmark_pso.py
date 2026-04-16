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
import time

#--- MAIN ---------------------------------------------------------------------+

class Particle:
    def __init__(self, bounds):
        self.position_i=[]          # particle position
        self.velocity_i=[]          # particle velocity
        self.pos_best_i=[]          # best position individual
        self.err_best_i=float('inf')         # best error individual
        self.err_i=float('inf')              # error individual

        for i in range(0,num_dimensions):
            v_max = (bounds[i][1] - bounds[i][0]) * 1
            self.velocity_i.append(uniform(-v_max, v_max))
            self.position_i.append(uniform(bounds[i][0], bounds[i][1]))

    # evaluate current fitness
    def evaluate(self,costFunc):
        self.err_i=costFunc(self.position_i)

        # check to see if the current position is an individual best
        if self.err_i<self.err_best_i:
            self.pos_best_i=self.position_i.copy()
            self.err_best_i=self.err_i
                    
    # update new particle velocity
    def update_velocity(self,pos_best_g, bounds):
        w=0.5       # constant inertia weight 
        c1=1        # cognative constant
        c2=2        # social constant
        
        for i in range(num_dimensions):
            r1=random()
            r2=random()
            
            vel_cognitive=c1*r1*(self.pos_best_i[i]-self.position_i[i])
            vel_social=c2*r2*(pos_best_g[i]-self.position_i[i])
            self.velocity_i[i]=w*self.velocity_i[i]+vel_cognitive+vel_social

            v_max = (bounds[i][1] - bounds[i][0]) * 1
            self.velocity_i[i] = max(-v_max, min(v_max, self.velocity_i[i]))

    # update the particle position 
    def update_position(self,bounds):
        for i in range(num_dimensions):
            self.position_i[i]=self.position_i[i]+self.velocity_i[i]
            
            # adjust maximum position if necessary
            if self.position_i[i]>bounds[i][1]:
                self.position_i[i]=bounds[i][1]

            # adjust minimum position if neseccary
            if self.position_i[i]<bounds[i][0]:
                self.position_i[i]=bounds[i][0]
        
        
def minimize(costFunc, bounds, num_particles, maxiter, verbose=False):
    start = time.time()
    global num_dimensions

    num_dimensions=len(bounds)
    err_best_g = float('inf')
    pos_best_g = []                  # best position for group
    history = []

    # establish the swarm
    swarm=[]
    for i in range(0,num_particles):
        particle = Particle(bounds)
        swarm.append(particle)

    for i in range(maxiter):
        if verbose: print(f'iter: {i:>4d}, best solution: {err_best_g:10.6f}')
            
        # cycle through particles in swarm and evaluate fitness
        for p in swarm:
            p.evaluate(costFunc)

            # determine if current particle is the best (globally)
            if p.err_i<err_best_g:
                pos_best_g=list(p.position_i)
                err_best_g=float(p.err_i)
        
        # cycle through swarm and update velocities and position
        for p in swarm:
            p.update_velocity(pos_best_g, bounds)
            p.update_position(bounds)

        history.append(err_best_g)

    # print final results
    if verbose  and err_best_g != float('inf'):
        print('\nFINAL SOLUTION:')
        print(f'   > {pos_best_g}')
        print(f'   > {err_best_g}\n')

    pso_time = time.time() - start
    return err_best_g, pos_best_g, pso_time, history

#--- END ----------------------------------------------------------------------+

# i = 0
# while i < maxiter:

#     for j in range(0, num_particles):
#         swarm[j].evaluate(costFunc)
#         if swarm[j].err_i < err_best_g:
#             pos_best_g = list(swarm[j].position_i)
#             err_best_g = float(swarm[j].err_i)

#     if verbose:
#         print(f'iter: {i+1:>4d}, best solution: {err_best_g:10.6f}')

#     for j in range(0, num_particles):
#         swarm[j].update_velocity(pos_best_g, bounds)
#         swarm[j].update_position(bounds)

#     i += 1
#     history.append(err_best_g)