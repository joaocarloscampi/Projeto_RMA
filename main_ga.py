import numpy as np
from ypstruct import structure
import ga
import simulation

def cost_func(t,theta,y):

    sum_cost = 0
    K_t = 10
    K_p = 5
    K_d = 2
    for i in range(5):
        sum_cost += (K_t*t[i] + K_p*theta[i]**2 + K_d*y[i]**2)

    return sum_cost

# Define univector constants
univector = structure()
univector.cost = cost_func
univector.nvar = 3
univector.varmin = 0
univector.varmax = 10

# Define param constants

params = structure()
params.maxit = 500
params.npop = 10

# Run genetic algorithm 

out = ga.run(univector, params)