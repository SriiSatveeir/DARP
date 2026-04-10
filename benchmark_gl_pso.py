from random import random, uniform
import time

class Particle:
    def __init__(self, bounds):
        self.position_i = []
        self.velocity_i = []
        self.pos_best_i = []
        self.err_best_i = float('inf')
        self.err_i = float('inf')

        for i in range(len(bounds)):
            v_max = (bounds[i][1] - bounds[i][0]) * 1
            self.velocity_i.append(uniform(-v_max, v_max))
            self.position_i.append(uniform(bounds[i][0], bounds[i][1]))

    def evaluate(self, costFunc):
        self.err_i = costFunc(self.position_i)
        if self.err_i < self.err_best_i:
            self.pos_best_i = self.position_i.copy()
            self.err_best_i = self.err_i

    def update_velocity(self, pos_best_g, global_best_error, pbest_avg, bounds):
        eps = 1e-12

        for i in range(len(self.position_i)):
            gbest_i = pos_best_g[i]
            pbest_i = self.pos_best_i[i]
            x_i = self.position_i[i]

            # FITNESS values (correct)
            fit_pbest = self.err_best_i
            fit_gbest = global_best_error

            # GLBest equations
            w = 1.1 - (fit_gbest / (pbest_avg + eps))
            c = 1.0 + (fit_gbest / (fit_pbest + eps))

            # Mild clamping (optional but safe)
            w = max(0.4, min(0.9, w))
            c = max(1.0, min(2.5, c))

            r = random()
            self.velocity_i[i] = w * self.velocity_i[i] + c * r * (pbest_i + gbest_i - 2 * x_i)

            # clip velocity
            v_max = (bounds[i][1] - bounds[i][0]) * 1
            self.velocity_i[i] = max(-v_max, min(v_max, self.velocity_i[i]))

    def update_position(self, bounds):
        for i in range(len(self.position_i)):
            self.position_i[i] += self.velocity_i[i]
            self.position_i[i] = max(bounds[i][0], min(bounds[i][1], self.position_i[i]))


def minimize(costFunc, bounds, num_particles, maxiter, verbose=False):
    start = time.time()
    num_dimensions = len(bounds)
    err_best_g = float('inf')
    pos_best_g = []
    history = []

    swarm = [Particle(bounds) for _ in range(num_particles)]
    

    for it in range(maxiter):
        # evaluate
        for p in swarm:
            p.evaluate(costFunc)
            if p.err_i < err_best_g:
                err_best_g = p.err_i
                pos_best_g = p.position_i.copy()

        # compute pbest average
        pbest_avg = sum(p.err_best_i for p in swarm) / num_particles

        # velocity & position updates
        for p in swarm:
            p.update_velocity(pos_best_g, err_best_g, pbest_avg, bounds)
            p.update_position(bounds)

        history.append(err_best_g)
        if verbose:
            print(f"Iter {it:>4d}, Best Fitness: {err_best_g:.6e}")

    pso_time = time.time() - start
    return err_best_g, pos_best_g, pso_time, history