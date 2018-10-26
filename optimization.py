from simulation import Simulation
import numpy as np
from operator import itemgetter

from deap import base, creator
import random
from deap import tools





MIN_PERIOD = 20
MAX_PERIOD = 180
MIN_PHASE_DISTRIBUTION = 0.1
MAX_PHASE_DISTRIBUTION = 0.9
MIN_OFFSET = 0
MAX_OFFSET = MAX_PERIOD - MIN_PERIOD

SEED = 2

SIMULATION_BUDGET = 50

GLOBAL_DEBUG_LEVEL = 3
DEBUG_PRINT_ON = True









simulation = Simulation()
simulation.setupSimulation()
nrIntersections = simulation.getNrIntersections()
random.seed(SEED)

bestResultLog = []

defaultInputSettings = {
    'phaseDistributions': [0.5] * nrIntersections,
    'period': 60,
    'offsets': [0] * nrIntersections
}

defaultInputSettingsNp = np.array(
    [
        np.array( [0.5] * nrIntersections ),
        60,
        np.array( [0] * nrIntersections )
    ]
)








#############################    GENETIC ALGORTIHM NEW, USING DEAP   #############################
toolbox = base.Toolbox()

GAbestResult = -1
GAbestResultInputSettings = -1
GAbestResultList = []

GA_POPULATION = 6

# gaussian mutation
MUTATION_MU = 0.5
MUTATION_SIGMA = 0.2

# polynomial mutation
POLYNOMIAL_ETA = 0.5
POLYNOMIAL_LOW = 0
POLYNOMIAL_HIGH = 1

MUTATION_PROBABILITY = 0.2
CROSSOVER_PROBABILITY = 0.5

SINGLE_ELEMENT_MUTATION_PROBABILITY = 0.4 # WAS 0.1
SINGLE_ELEMENT_CROSSOVER_PROBABILITY = 0.5


def evaluteGeneticAlgortihm(individual):
    inputSettings = convertListToInputSettings(individual)

    result = simulation.runAdjustedSimulation(inputSettings)

    possiblyStoreBestResult(result, inputSettings)

    return result,




def possiblyStoreBestResult(result, inputSettings):
    global GAbestResult
    global GAbestResultInputSettings
    global bestResultLog

    if result > GAbestResult:
        GAbestResult = result
        GAbestResultInputSettings = inputSettings + 0

        print('New best found')
        print('Result: \t', result)
        print('inputSettings:\n', inputSettings)

    bestResultLog.append(GAbestResult)



def setupGeneticAlgorithm():
    global toolbox
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    IND_SIZE = getNrInputs()

    toolbox.register("attribute", random.random)
    toolbox.register("individual", tools.initRepeat, creator.Individual,
                     toolbox.attribute, n=IND_SIZE)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("mate", tools.cxUniform, indpb = SINGLE_ELEMENT_CROSSOVER_PROBABILITY)
    #toolbox.register("mutate", tools.mutGaussian, mu=MUTATION_MU, sigma=MUTATION_SIGMA, indpb=SINGLE_ELEMENT_MUTATION_PROBABILITY)
    toolbox.register("mutate", tools.mutPolynomialBounded, eta = POLYNOMIAL_ETA, low = POLYNOMIAL_LOW, up = POLYNOMIAL_HIGH, indpb = MUTATION_PROBABILITY)
    toolbox.register("select", tools.selTournament, tournsize=3)
    toolbox.register("evaluate", evaluteGeneticAlgortihm)



def fullGeneticAlgorithm():
    setupGeneticAlgorithm()

    global toolbox
    pop = toolbox.population(n=GA_POPULATION)
    CXPB, MUTPB = CROSSOVER_PROBABILITY, MUTATION_PROBABILITY

    # Evaluate the entire population
    fitnesses = map(toolbox.evaluate, pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    g = 0
    while simulation.getNrSimulationsRun() < SIMULATION_BUDGET:
        g += 1
        printDebug('Generation ', g, end='\t\t')
        printProgress(simulation.getNrSimulationsRun() / SIMULATION_BUDGET)
        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2) # todo: kan hier adaptive gedeelte byvoeg
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant) # todo: kan hier eenvoudige adaptive gedeelte byvoeg deur mutation probability te laat afhang van die child se fitness values
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        # The population is entirely replaced by the offspring
        pop[:] = offspring

    return pop

def geneticAlgorithmMain():
    fullGeneticAlgorithm()
    printResults('Genetic Algortihm', GAbestResultInputSettings)




#############################    GENETIC ALGORTIHM OLD   #############################

def generateRandomInputSettings():
    inputSettings = defaultInputSettings

    inputSettings['phaseDistributions'] = [np.random.uniform(MIN_PHASE_DISTRIBUTION, MAX_PHASE_DISTRIBUTION) for i in range(nrIntersections)]
    inputSettings['period'] = np.random.uniform(MIN_PERIOD, MAX_PERIOD)
    inputSettings['offsets'] = [np.random.uniform(0, defaultInputSettings['period']) for i in range(nrIntersections)]

    return inputSettings


def generateRandomInputSettingsNp():
    newInputSettings = np.array(
        [
            np.array([np.random.uniform(MIN_PHASE_DISTRIBUTION, MAX_PHASE_DISTRIBUTION) for i in range(nrIntersections)]),
            np.random.uniform(MIN_PERIOD, MAX_PERIOD),
            np.array([np.random.uniform(0, defaultInputSettings['period']) for i in range(nrIntersections)])
        ]
    )

    return newInputSettings


def mate(individual1, individual2):
    CHANCE_THAT_GENES_EXCHANGE = 0.2

    child1 = individual1 + 0
    child2 = individual2 + 0

    for inputTypeIndex in  range(len(individual1)):
        if inputTypeIndex == 0:
            for i in range(len(individual1[inputTypeIndex])):
                if np.random.random() < CHANCE_THAT_GENES_EXCHANGE:
                    tempValue = child2[inputTypeIndex][i]
                    chidl2[inputTypeIndex][i] = child1[inputTypeIndex][i]
                    child1[inputTypeIndex][i] = tempValue
        elif inputTypeIndex == 1:
            if np.random.random() < CHANCE_THAT_GENES_EXCHANGE:
                tempValue = child2[inputTypeIndex][i]
                chidl2[inputTypeIndex][i] = child1[inputTypeIndex][i]
                child1[inputTypeIndex][i] = tempValue
        else:
            for i in range(len(individual1[inputTypeIndex])):
                if np.random.random() < CHANCE_THAT_GENES_EXCHANGE:
                    tempValue = child2[inputTypeIndex][i]
                    chidl2[inputTypeIndex][i] = child1[inputTypeIndex][i]
                    child1[inputTypeIndex][i] = tempValue

    return child1, child2











#############################    EXHUASTIVE SEARCH    #############################

ESbestResult = 0
ESbestResultInputSetting = defaultInputSettingsNp + 0

def fullExhuastiveSearch():
    SIMULATIONS_PER_INPUT = int(round(SIMULATION_BUDGET / getNrInputs(), 0))

    inputSettings = defaultInputSettingsNp + 0

    for inputType in range(len(inputSettings)):
        if inputType == 0:      # phase distribution
            for intersectionIndex in range(len(inputSettings[inputType])):
                inputSettings = singleExhuastiveSearch(MIN_PHASE_DISTRIBUTION, MAX_PHASE_DISTRIBUTION, SIMULATIONS_PER_INPUT, inputSettings, inputType, intersectionIndex)
                printProgressGlobal()
        elif inputType == 1:    # cycle length
            inputSettings = singleExhuastiveSearch(MIN_OFFSET, MAX_OFFSET, SIMULATIONS_PER_INPUT, inputSettings, inputType, None)
            printProgressGlobal()
        else:                   # offset
            for intersectionIndex in range(len(inputSettings[inputType])):
                inputSettings = singleExhuastiveSearch(MIN_OFFSET, MAX_OFFSET, SIMULATIONS_PER_INPUT, inputSettings, inputType, intersectionIndex)
                printProgressGlobal()
    '''
    printDebug('Exhuastive search completed.\nInputSettings:\n', inputSettings, debugLevel=5)
    printDebug('Simulation run:', simulation.getNrSimulationsRun())
    printDebug('Result: ', ESbestResult)
    '''

    printResults('Exhaustive Search', inputSettings)



def singleExhuastiveSearch(MIN, MAX, SIMULATIONS_PER_INPUT, inputSettings, inputType, intersectionIndex):
    bestResult = -1
    bestResultInputSetting = -1

    for iteration in range(SIMULATIONS_PER_INPUT):
        newInputSetting = MIN + (MAX - MIN) / SIMULATIONS_PER_INPUT * (iteration + 1)

        if intersectionIndex is None:
            inputSettings[inputType] = newInputSetting
        else:
            inputSettings[inputType][intersectionIndex] = newInputSetting

        result = evaluate(inputSettings)

        if result > bestResult:
            bestResultInputSetting = newInputSetting + 0

            global ESbestResult
            if result > ESbestResult:
                ESbestResult = result

    if intersectionIndex is None:
        inputSettings[inputType] = bestResultInputSetting + 0
    else:
        inputSettings[inputType][intersectionIndex] = bestResultInputSetting + 0

    return inputSettings













#############################    NELDER-MEAD    #############################


def nelderMeadOneDimension():
    PHASE_DISTRIBUTION_INTIAL_DIFFERENCE = 0.15
    SIMULATION_BUDGET = 10
    REFLECTION_FACTOR = 1
    EXPANSION_FACTOR = 2
    CONTRACTION_FACTOR = 0.5

    point1 = defaultInputSettingsNp + 0
    point2 = point1 + 0 # quick and dirty
    point2[0][0] -= PHASE_DISTRIBUTION_INTIAL_DIFFERENCE

    points= [[point1, 0], [point2, 0]]
    for p in points:
        p[1] = simulation.runAdjustedSimulation(p[0])
        sorted(points, key=itemgetter(1))

    nrSimulations = 1
    while nrSimulations < SIMULATION_BUDGET:

        print('Points:\t', points)

        x_reflected = [points[0][0] + REFLECTION_FACTOR * (points[0][0] - points[1][0])]
        #print('x-reflected:\t', x_reflected[0])
        x_reflected.append(simulation.runAdjustedSimulation(x_reflected[0]))
        #print('x-r-result:\t', x_reflected[1])

        if x_reflected[1] > points[0][1]: # new point is best so far
            x_expanded = [points[0][0] + EXPANSION_FACTOR * (x_reflected[0] - points[0][0])]
            x_expanded.append(simulation.runAdjustedSimulation(x_expanded[0]))

            if x_expanded[1] > x_reflected[1]:
                points.pop(1)
                points.insert(0, x_expanded)
                print('x_expanded:\t\t', x_expanded)
            else:
                points.pop(1)
                points.insert(0, x_reflected)
                print('x_reflected:\t\t', x_reflected)

        else: # contract
            if  x_reflected[1] > points[1][1]:
                x_contracted = [points[0][0] + CONTRACTION_FACTOR * (x_reflected[0] - points[0][0])]
                x_contracted.append(simulation.runAdjustedSimulation(x_contracted[0]))
                points.pop(1)
                points.insert(0, x_contracted)
                print('x_contracted (to x_reflected):\t\t', x_contracted)
            else:
                x_contracted = [points[0][0] + CONTRACTION_FACTOR * (points[1][0] - points[0][0])]
                x_contracted.append(simulation.runAdjustedSimulation(x_contracted[0]))
                points.pop(1)
                points.insert(0, x_contracted)
                print('x_contracted (to x_2):\t\t', x_contracted)


        nrSimulations += 1



def singleNelderMead(MIN, MAX, SIMULATIONS_PER_INPUT, inputSettings, inputType, intersectionIndex):
    INITIAL_DIFFERENCE = 3 / 16 * (MAX-MIN)
    REFLECTION_FACTOR = 1
    EXPANSION_FACTOR = 2
    CONTRACTION_FACTOR = 0.5

    point1 = inputSettings + 0
    point2 = point1 + 0 # quick and dirty

    initialNrSimulations = simulation.getNrSimulationsRun()

    if intersectionIndex is None:
        point2[inputType] -= INITIAL_DIFFERENCE
    else:
        point2[inputType][intersectionIndex] -= INITIAL_DIFFERENCE

    points= [[point1, 0], [point2, 0]]
    for p in points:
        p[1] = evaluate(p[0])
        sorted(points, key=itemgetter(1))

    while simulation.getNrSimulationsRun() - initialNrSimulations < SIMULATIONS_PER_INPUT:

        printDebug('Points:\t', points)

        x_reflected = [points[0][0] + REFLECTION_FACTOR * (points[0][0] - points[1][0])]
        #print('x-reflected:\t', x_reflected[0])
        x_reflected.append(evaluate(x_reflected[0]))
        #print('x-r-result:\t', x_reflected[1])

        if x_reflected[1] > points[0][1]: # new point is best so far
            x_expanded = [points[0][0] + EXPANSION_FACTOR * (x_reflected[0] - points[0][0])]
            x_expanded.append(evaluate(x_expanded[0]))

            if x_expanded[1] > x_reflected[1]:
                points.pop(1)
                points.insert(0, x_expanded)
                printDebug('x_expanded:\t\t', x_expanded)
            else:
                points.pop(1)
                points.insert(0, x_reflected)
                printDebug('x_reflected:\t\t', x_reflected)

        else: # contract
            if  x_reflected[1] > points[1][1]:
                x_contracted = [points[0][0] + CONTRACTION_FACTOR * (x_reflected[0] - points[0][0])]
                x_contracted.append(evaluate(x_contracted[0]))
                points.pop(1)
                points.insert(0, x_contracted)
                printDebug('x_contracted (to x_reflected):\t\t', x_contracted)
            else:
                x_contracted = [points[0][0] - CONTRACTION_FACTOR * (points[0][0] - points[1][0])]
                x_contracted.append(evaluate(x_contracted[0]))
                points.pop(1)
                points.insert(0, x_contracted)
                printDebug('x_contracted (to x_2):\t\t', x_contracted)

    bestPoint = points[0][0]
    return bestPoint


def fullNelderMead():
    SIMULATIONS_PER_INPUT = int(round(SIMULATION_BUDGET / getNrInputs(), 0))

    inputSettings = defaultInputSettingsNp + 0

    for inputType in range(len(inputSettings)):
        if inputType == 0:      # phase distribution
            for intersectionIndex in range(len(inputSettings[inputType])):
                inputSettings = singleNelderMead(MIN_PHASE_DISTRIBUTION, MAX_PHASE_DISTRIBUTION, SIMULATIONS_PER_INPUT, inputSettings, inputType, intersectionIndex)
                printProgressGlobal()
        elif inputType == 1:    # cycle length
            inputSettings = singleNelderMead(MIN_OFFSET, MAX_OFFSET, SIMULATIONS_PER_INPUT, inputSettings, inputType, None)
            printProgressGlobal()
        else:                   # offset
            for intersectionIndex in range(1, len(inputSettings[inputType])):
                inputSettings = singleNelderMead(MIN_OFFSET, inputSettings[1], SIMULATIONS_PER_INPUT, inputSettings, inputType, intersectionIndex)
                printProgressGlobal()

    # printDebug('Nelder Mead completed.\nInputSettings:\n', inputSettings, debugLevel=5)
    printResults('Nelder Mead', inputSettings)




#############################    GENERAL    #############################


def printProgress(percentage):
    printDebug('Optimization is ', round(percentage * 100, 1), ' %...')


def printProgressGlobal():
    printProgress(simulation.getNrSimulationsRun() / SIMULATION_BUDGET)


def testResult(inputSettings):
    result1 = simulation.runAdjustedSimulation(inputSettings)

    print (result1)


def getNrInputs():
    return nrIntersections * 2 + 1 - 1 # to account for master cycle length and no parameter for last  intersection


def printDebug(*arg, debugLevel = 0, end='\n'):
    if DEBUG_PRINT_ON and debugLevel >= GLOBAL_DEBUG_LEVEL:
        for msg in arg:
            print (msg, end='')
        print(end=end)



def convertListToInputSettings(listIn):
    inputSettings = np.array([
        np.array([
                    convert0to1toRealWorld(listIn[i], MIN_PHASE_DISTRIBUTION, MAX_PHASE_DISTRIBUTION)
                    for i in range(nrIntersections)
                  ]),
        int(round(convert0to1toRealWorld(listIn[nrIntersections], MIN_PERIOD, MAX_PERIOD))),
        np.array(
                  [0] +
                  [
                    convert0to1toRealWorld(listIn[i], MIN_OFFSET, MAX_OFFSET)
                    for i in range(nrIntersections + 1, nrIntersections * 2)
                  ])
    ])

    return inputSettings


def convert0to1toRealWorld(float0to1, MIN, MAX):
    realValue = float0to1 * (MAX - MIN) + MIN
    return realValue


def testFunction():
    bestResultLog.append(1)

def evaluate(inputSettings):
    result = simulation.runAdjustedSimulation(inputSettings)
    possiblyStoreBestResult(result, inputSettings)
    return result

def printResults(algorithm, bestInputSettings):
    print('---------- ' , algorithm, 'results ----------')
    print('Best solution:  \t', bestInputSettings)
    print('Result:         \t', bestResultLog[len(bestResultLog) - 1])
    print('Simulations run:\t', simulation.getNrSimulationsRun())
    print('Result log:     \t', bestResultLog)



#############################    MAIN CODE STARTS HERE    #############################

# nelderMeadOneDimension()

# singleExhuastiveSearch(MIN_PHASE_DISTRIBUTION, MAX_PHASE_DISTRIBUTION, 10, defaultInputSettingsNp, 0, 0)
# printDebug(ESbestResult)
# print(generateProbablisticInputSettings())

# fullNelderMead()

#geneticAlgorithmMain()


'''
inputSettingsResult = np.array([
        np.array([0.54291661, 0.77746478]),
        129 ,
        np.array([150.67387661, 132.13265422])
    ])
testResult(inputSettingsResult)
'''

class Optimization:

    def optimizeAndLog(self, algorithm, simulationBudget = None):
        global bestResultLog
        global SIMULATION_BUDGET

        if simulationBudget is not None:
            SIMULATION_BUDGET = simulationBudget

        bestResultLog = []

        if algorithm == 'Genetic Algorithm':
            geneticAlgorithmMain()
        elif algorithm == 'Nelder Mead':
            fullNelderMead()
        elif algorithm == 'Exhaustive Search':
            fullExhuastiveSearch()

        return bestResultLog

    def testResult(self, inputSettings):
        testResult(inputSettings)










'''
inputSettingsResultSolutionProvided = np.array([
        np.array([ 0.3125,  0.425 ]),
        65.625,
        np.array([ 0,   12.3046875 ])
    ])

inputSettingsResultAltered = np.array([
        np.array([ 0.38,  0.38 ]),
        65.625,
        np.array([ 0,   8.3046875 ])
    ])

testResult(inputSettingsResultAltered)
'''





