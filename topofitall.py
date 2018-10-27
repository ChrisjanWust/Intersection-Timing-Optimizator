from optimization import Optimization
from simulation import Simulation
import numpy as np


inputSettings = np.array([
        np.array([ 0.89777178,  0.86754191]),
        29,
        np.array([ 0,   93.47738382 ])
    ])

inputSettings = np.array([
        np.array([ 0.4,  0.6]),
        40,
        np.array([ 0,   30 ])
    ])

'''
optimization = Optimization()
optimization.testResult(inputSettingsResult)
'''


simulation = Simulation()



mss = simulation.runAdjustedSimulation(inputSettings)
print (mss)