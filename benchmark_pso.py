#------------------------------------------------------------------------------+
#
#	Nathan A. Rooy
#	Simple Particle Swarm Optimization (PSO) with Python
#	Last update: 2018-JAN-26
#	Python 3.6
#
#------------------------------------------------------------------------------+

#--- IMPORT DEPENDENCIES ------------------------------------------------------+

import time
from random import random
from random import uniform
from benchmark_functions import BF

#--- MAIN ---------------------------------------------------------------------+

class Particle:
    def __init__(self, bounds):
        self.position_i=[]          # particle position
        self.velocity_i=[]          # particle velocity
        self.pos_best_i=[]          # best position individual
        self.err_best_i=-1          # best error individual
        self.err_i=-1               # error individual

        for i in range(len(bounds)):
            self.velocity_i.append(uniform(-1,1))
            self.position_i.append(uniform(bounds[i][0], bounds[i][1]))

    # evaluate current fitness
    def evaluate(self,costFunc):
        self.err_i=costFunc(self.position_i)

        # check to see if the current position is an individual best
        if self.err_i<self.err_best_i or self.err_best_i==-1:
            self.pos_best_i=self.position_i.copy()
            self.err_best_i=self.err_i
                    
    # update new particle velocity
    def update_velocity(self,pos_best_g):
        w=0.5       # constant inertia weight (how much to weigh the previous velocity)
        c1=1        # cognative constant
        c2=2        # social constant
        
        for i in range(len(self.position_i)):
            r1=random()
            r2=random()
            
            vel_cognitive=c1*r1*(self.pos_best_i[i]-self.position_i[i])
            vel_social=c2*r2*(pos_best_g[i]-self.position_i[i])
            self.velocity_i[i]=w*self.velocity_i[i]+vel_cognitive+vel_social

    # update the particle position based off new velocity updates
    def update_position(self,bounds):
        for i in range(len(bounds)):
            self.position_i[i]=self.position_i[i]+self.velocity_i[i]
            
            # adjust maximum position if necessary
            if self.position_i[i]>bounds[i][1]:
                self.position_i[i]=bounds[i][1]

            # adjust minimum position if neseccary
            if self.position_i[i]<bounds[i][0]:
                self.position_i[i]=bounds[i][0]
        
        
def minimize(costFunc, bounds, num_particles, maxiter, verbose=False):
    start_time = time.time()

    swarm = [Particle(bounds) for _ in range(num_particles)]

    err_best_g = float('inf')
    pos_best_g = [0.0] * len(bounds)

    history = []

    for iter in range(maxiter):
        for particle in swarm:
            particle.evaluate(costFunc)

            if particle.err_i < err_best_g:
                pos_best_g = particle.position_i.copy()
                err_best_g = particle.err_i

        history.append(err_best_g)

        for particle in swarm:
            particle.update_velocity(pos_best_g)
            particle.update_position(bounds)

        if verbose:
            print(f"iter: {iter:>4d}, best: {err_best_g:.6f}")

    pso_time = time.time() - start_time

    # print final results
    if verbose:
        print('\nFINAL SOLUTION:')
        print(f'   > {pos_best_g}')
        print(f'   > {err_best_g}\n')

    return err_best_g, pos_best_g, history

#--- END ----------------------------------------------------------------------+