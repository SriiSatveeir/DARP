import math
#-------------------------
# Sphere function
#-------------------------
def sphere(position):
    total=0.0
    for i in range(len(position)):
        xi = position[i]
        total+= xi**2
    return total

sphere_dim = 30
sphere_bounds = [(-10, 10)] * sphere_dim
    
#-------------------------
# Rosenbrock function
#-------------------------
def rosenbrock(position):
    total=0.0
    for i in range(len(position)-1):
        xi = position[i]
        xi2 = position[i+1]
        total+= 100*(xi2 - xi**2)**2 + (xi-1)**2
    return total

rosenbrock_dim = 7
rosenbrock_bounds = [(-5, 10)] * rosenbrock_dim

#-------------------------
# Rastrigin function
#-------------------------
def rastrigin(position):
    total=0.0
    for i in range(len(position)):
        xi = position[i]
        total+= (xi**2) - (10 * math.cos(2*math.pi*xi)) + 10
    return total

rastrigin_dim = 10
rastrigin_bounds = [(-5.12, 5.12)] * rastrigin_dim