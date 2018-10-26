from optimization import Optimization
from simulation import Simulation
import numpy as np


inputSettingsResult = np.array([
        np.array([ 0.89777178,  0.86754191]),
        29,
        np.array([ 0,   93.47738382 ])
    ])


'''
optimization = Optimization()
optimization.testResult(inputSettingsResult)
'''


simulation = Simulation()



simulation.runAdjustedSimulation(inputSettingsResult)