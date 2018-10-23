from scenario import Scenario
import time as time_lib
from timer import Timer
from display import Display
import numpy as np


# INITIALIZE CONSTANTS
SPEED_LIMIT = 15 # upper speed limit (m/s)
MAX_DECELERATION = 2.5 # (m/s^2)
ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)
INTERSECTION_SPEED = 8
CAR_LENGTH = 6
MIN_CAR_LENGTH_BETWEEN_CARS = 1.5
ROAD_WIDTH = 6 # declared in display as well

# CONSTRAINTS
CONSTRAINTS_ACTIVE = False
MAX_ENTRY_QUEUE = 4
CHECK_CONSTRAINTS_TIMESTEP = 10
MAX_VEHICLE_DENSITY = 0.5 # (veh / m) . 0.15 still activates sometimes


SCENARIO_NUMBER = 1
SEED = 5 # 5 is not bad
SIMULATIONS_PER_CHANGE = 3
POINTS_SIMULATED = 9

STOP_DISTANCE_FROM_LINE = 2 # (m) = meters from intersection line to where car stops

# Mean System Speed
CHECK_MEAN_SYSTEM_SPEED_TIMESTEP = 10 * TIME_STEP # (number of timesteps) = only check every X timesteps whether the mean speed has aggregated
MEAN_SYSTEM_SPEED_CONSISTENCY_PERIOD = 60 * 2 # (S) = time during which mean system speed must stay consistent
MEAN_SYSTEM_SPEED_AGGREGATION_PERCENTAGE = 10 # (%) =  current mean system speed must within this percentage of old mean system speed before measuring can begin
MEASURING_PERIOD = 60 * 5 # (s) = 5 mins
AVERAGE_NOT_MEDIAN = True


# debugging
DEBUG_PRINT_ON = True
GLOBAL_DEBUG_LEVEL = 5
DISPLAY_ON = False
SPEED_IN_KMPERHOUR = True


# initialize public variables
cars = []
links = []
intersections = []
entries = []
time = 0
systemDistanceTravelled = 0
systemTimeTravelled = 0
meanSystemSpeedLog = [-1.0] * int(MEAN_SYSTEM_SPEED_CONSISTENCY_PERIOD / CHECK_MEAN_SYSTEM_SPEED_TIMESTEP)
display = Display()
timer = Timer()
measuringStarted = 0 # (s) = time measuring was started
previousProcessingTimestamp = 0
carsArrived = 0

#constraintVariables
phaseDistributionUpperLimit = 0.9
phaseDistributionLowerLimit = 0.1
cyclePeriodUpperLimit = 180
cyclePeriodLowerLimit = 30

# needs serious attention
currentPhaseDistribution = 0.5 # value represent the starting phase distribution
bestPhaseDistribution = 0.1


# gradient
gradientStep = 0.2

# optimization
nrSimulationRuns = 0


def seedEntries(seed):
    np.random.seed(seed)
    for entry in entries:
        entry.seed(np.random.randint(0, 1000))


def setupSimulation():
    global links
    global intersections
    global entries
    global meanSystemSpeedLog

    current_scenario = Scenario.loadScenario(SCENARIO_NUMBER)
    links = current_scenario['links']
    intersections = current_scenario['intersections']
    entries = current_scenario['entries']




    if DISPLAY_ON:
        display.setup(links, intersections, 1 / TIME_STEP)

    meanSystemSpeedLog = [-1.0] * int(MEAN_SYSTEM_SPEED_CONSISTENCY_PERIOD / CHECK_MEAN_SYSTEM_SPEED_TIMESTEP)



def resetSimulation():
    global cars
    global time
    global systemTimeTravelled
    global systemDistanceTravelled
    global measuringStarted
    global meanSystemSpeedLog
    global carsArrived

    cars = []
    time = 0
    systemDistanceTravelled = 0
    systemTimeTravelled = 0

    carsArrived = 0

    # measuringStarted = 0 # should actually be included, but getting irregular results with different starting times

    #clearMeanSystemsLog()
    meanSystemSpeedLog = [-1.0] * int(MEAN_SYSTEM_SPEED_CONSISTENCY_PERIOD / CHECK_MEAN_SYSTEM_SPEED_TIMESTEP)



def clearMeanSystemsLog():
    global meanSystemSpeedLog
    for i in range(len(meanSystemSpeedLog)):
        if i < MEAN_SYSTEM_SPEED_CONSISTENCY_PERIOD / CHECK_MEAN_SYSTEM_SPEED_TIMESTEP:
            meanSystemSpeedLog[i] = -1
        else:
            meanSystemSpeedLog.pop(i)

def speedToString(speed):
    if (SPEED_IN_KMPERHOUR):
        return str(round(speed * 3.6,2)) + str('km/h')
    return str(round(speed,2)) + str('m/s')

def printDebug(*arg, debugLevel = 0):
    if DEBUG_PRINT_ON and debugLevel >= GLOBAL_DEBUG_LEVEL:
        for msg in arg:
            print (msg, end='')



def runSingleTimestep():
    global cars
    global links
    global intersections
    global time
    global allHasReachedDestination
    global systemTimeTravelled
    global systemDistanceTravelled
    global carsArrived

    allHasReachedDestination = True

    carsToBeRemoved = []

    generateNewCars()

    for i in range(len(cars)):

        currentLink, distanceInLink, speed, direction = cars[
            i].getPositionAndSpeed()  # could get this with separate get methods

        printDebug("\n", time, "s  ", "\tcar: ", i,
                   "\tcurrentLink: ", currentLink,
                   "\tdistanceInLink: ", round(distanceInLink, 2),
                   " \tspeed: ", speedToString(speed))

        if not cars[i].hasReachedDestination():  # before doing calculations, check if car has already reached destination # should rather delete cars that has reached destination

            allHasReachedDestination = False

            # FINDING CLOSEST OBJECT (can be an intersection or car)

            nextObject_distance, nextObject_speed = findNextObstacle(currentLink, distanceInLink, direction)

            # move
            distanceInLink = cars[i].move2withAggressiveness(nextObject_distance, nextObject_speed) # move2 is reliable @ 11 Oct

            # routing
            if (distanceInLink > links[currentLink].getDistance()):
                if (links[currentLink].getIntersection(direction) != -1): # change to another link
                    cars[i].addDistanceTravelled(links[currentLink].getDistance())

                    # done: use updated version below (which accomodates for intersections)
                    #cars[i].changeLink(intersections[links[currentLink].getIntersection(direction)].getLink(direction),
                    #                   distanceInLink - links[currentLink].getDistance(), direction)

                    cars[i].changeLink(intersections[links[currentLink].getIntersection(direction)].getLink(direction),
                                       distanceInLink - links[currentLink].getDistance() - ROAD_WIDTH, direction, time=time)

                    # print('Changing to  link', intersections[links[currentLink].getIntersection(direction)].getLink(direction), 'from intersection', links[currentLink].getIntersection(direction), end='')
                else: # car has reached destination
                    cars[i].addDistanceTravelled(distanceInLink)
                    cars[i].changeLink(-1,distanceInLink - links[currentLink].getDistance(), direction)
                    systemDistanceTravelled += cars[i].getDistanceTravelled()
                    systemTimeTravelled += cars[i].getTimeTravelled(time + TIME_STEP) # by this stage, move has already been executed
                    carsToBeRemoved.append(i)
                    carsArrived += 1

        else:
            printDebug("  \talready arrived")


    carsToBeRemoved.sort(reverse=True)
    for i in carsToBeRemoved:
        cars.pop(i)

    if DISPLAY_ON:
        display.update(cars, time, carsArrived, systemDistanceTravelled / (systemTimeTravelled + 0.00001))

    continueSimulation = True

    if time % CHECK_MEAN_SYSTEM_SPEED_TIMESTEP == 0:
        continueSimulation = checkMeanSystemSpeed()

    if CONSTRAINTS_ACTIVE:
        if time % CHECK_CONSTRAINTS_TIMESTEP == 0:
            constraintsOk = checkConstraints()
            if not constraintsOk:
                setPhaseDistributionLimit()
            continueSimulation = continueSimulation and constraintsOk
            print('SIMULATION CONSTRAINT REACHED')

            systemDistanceTravelled = systemDistanceTravelled/10

    time = round(time + TIME_STEP, 1)

    return continueSimulation

def generateNewCars():
    for entry in entries:
        entry.possiblyGenerateCar(TIME_STEP, time)

        if entry.getQueueLength(): # don't want to spend time on looping through all cars when unnecessary
            distanceTillNextCar = findNextObstacle(entry.getLink(), 0, entry.getDirection())[0]
            if distanceTillNextCar > 0:
                cars.append(entry.possiblyGetCar()) # there will definitely be a car since this is in "if entry.hasCarAvailable"



def checkIndividualSpeedInLinks():
    for car in cars:
        if not car.checkConstraintsOk(time):
            return False

    return True


def checkEntryQueueLengths():
    for entry in entries:
        if entry.getQueueLength() > MAX_ENTRY_QUEUE:
            return False

    return True

def checkCarDensity ():
    nrCarsInLinks = [0] * len(links)
    for car in cars:
        currentLink = car.getPositionAndSpeed()[0]
        nrCarsInLinks[currentLink] += 1

    i = 0
    for nrCarsInLink in nrCarsInLinks:
        if nrCarsInLink / links[i].getDistance() > MAX_VEHICLE_DENSITY:
            return False
        i += 1

    return True


def setPhaseDistributionLimit():
    global phaseDistributionUpperLimit
    global phaseDistributionLowerLimit

    if currentPhaseDistribution - phaseDistributionLowerLimit < phaseDistributionUpperLimit - currentPhaseDistribution:
        phaseDistributionLowerLimit = currentPhaseDistribution
        printDebug('Lower phase distribution limit set at ', currentPhaseDistribution, debugLevel=1)
    else:
        phaseDistributionUpperLimit = currentPhaseDistribution
        printDebug('Upper phase distribution limit set at ', currentPhaseDistribution, debugLevel=1)



def checkConstraints():
    constraintsOk = checkEntryQueueLengths() and checkIndividualSpeedInLinks() and checkCarDensity()
    return constraintsOk

def findNextObstacle(linkIndex, distanceInLink, direction):
    # initialize with maximum
    nextObjectDistance = 999
    nextObjectSpeed = SPEED_LIMIT

    nextIntersectionIndex = links[linkIndex].getIntersection(direction)
    if nextIntersectionIndex != -1:
        nextObjectDistance = links[linkIndex].getDistance() - distanceInLink - STOP_DISTANCE_FROM_LINE
        if intersections[nextIntersectionIndex].getStatus(time, direction) == 'GREEN':
            nextObjectSpeed = INTERSECTION_SPEED
        else:
            nextObjectSpeed = 0

    for car in cars:
        carLinkIndex, carDistanceInLink, carSpeed, carDirection = car.getPositionAndSpeed()

        # statements can be combined, but keep for now for readability
        if carLinkIndex == linkIndex and direction == carDirection:
            if distanceInLink < carDistanceInLink:
                if carDistanceInLink - distanceInLink - MIN_CAR_LENGTH_BETWEEN_CARS * CAR_LENGTH < nextObjectDistance:
                    nextObjectDistance = carDistanceInLink - distanceInLink - CAR_LENGTH * MIN_CAR_LENGTH_BETWEEN_CARS
                    nextObjectSpeed = carSpeed

    return nextObjectDistance, nextObjectSpeed


def runSingleSimulation():
    resetSimulation()
    runSimulation = True

    while runSimulation:
        runSimulation = runSingleTimestep()

def checkMeanSystemSpeed():
    global measuringStarted
    global systemTimeTravelled
    global systemDistanceTravelled
    global meanSystemSpeedLog

    if measuringStarted:
        if time - measuringStarted > MEASURING_PERIOD:
            printDebug("Measuing period is over at ", time, ".\nSimulation stopping...",
                       "\n", debugLevel=1)
            return False
    elif systemTimeTravelled > 0:
        newMeanSystemSpeed = systemDistanceTravelled/systemTimeTravelled
        meanSystemSpeedLog.append(systemDistanceTravelled/systemTimeTravelled)
        oldMeanSystemSpeed = meanSystemSpeedLog.pop(0)
        if oldMeanSystemSpeed * (1 - MEAN_SYSTEM_SPEED_AGGREGATION_PERCENTAGE / 100) < newMeanSystemSpeed < oldMeanSystemSpeed * (1 + MEAN_SYSTEM_SPEED_AGGREGATION_PERCENTAGE / 100):
            measuringStarted = time
            systemTimeTravelled = 0
            systemDistanceTravelled = 0

            printDebug("Measuring turned on at ", time, "\nOld mean system speed is ", oldMeanSystemSpeed, "\nCurrent mean system speed is ", newMeanSystemSpeed, '\n', debugLevel=1)

    # this is a hack
    if time == measuringStarted:
        systemTimeTravelled = 0
        systemDistanceTravelled = 0

    return True

def changeIntersectionTimings2(searchDirection, previousResult, newPoint, newResult, intersection):
    global currentPhaseDistribution

    if newResult < previousResult:
        searchDirection = - searchDirection
        printDebug("Changing search direction", debugLevel=1)

    newPoint = newPoint + searchDirection * 0.1

    # limiting
    if newPoint > .95:
        newPoint = .95
    elif newPoint < 0.05:
        newPoint = 0.05

    currentPhaseDistribution = newPoint

    intersection.setPhaseDistribution(currentPhaseDistribution)

def measureProcessingTime():
    global previousProcessingTimestamp
    newProcessingTimestamp = time_lib.process_time()
    elapsedProcessingTime = newProcessingTimestamp - previousProcessingTimestamp
    printDebug("Delta processing time: ", elapsedProcessingTime, 's\n', debugLevel=1)
    previousProcessingTimestamp = newProcessingTimestamp

def startProcessingTimer():
    time_lib.process_time()

def getNrIntersections():
    return len(intersections)

def applyInputSettingsNp(inputSettings):
    i = 0
    for intersection in intersections:
        intersection.setPhaseDistribution(inputSettings[0][i])
        intersection.setPeriod(inputSettings[1])
        intersection.setOffset(inputSettings[2][i])
        i += 1

def applyInputSettings(inputSettings):
    i = 0
    for intersection in intersections:
        intersection.setPhaseDistribution(inputSettings['phaseDistributions'][i])
        intersection.setPeriod(inputSettings['period'])
        intersection.setOffset(inputSettings['offsets'][i])
        i += 1

def runMeasurementSimulations():
    global currentPhaseDistribution

    resolution = 100
    currentPhaseDistribution = 1 / resolution
    setupSimulation()

    printDebug('Phase Distribution\tMean System Speed\n', debugLevel=5)

    for intersection in intersections:
        for i in range(resolution - 1):

            measuredMeanSystemSpeeds = []


            for j in range(SIMULATIONS_PER_CHANGE):
                seedEntries(SEED + j * 10)

                printDebug("\n\nStarting simulation..."
                           "\nPhaseDistribution: ", currentPhaseDistribution, "\n", debugLevel=1)
                # printDebug("meanSpeedsLog: ", meanSystemSpeedLog, "\n", selected=1)
                resetSimulation()
                runSingleSimulation()
                newMeasuredMeanSystemSpeed = systemDistanceTravelled / systemTimeTravelled
                printDebug("measuredSystemSpeed: ", newMeasuredMeanSystemSpeed, '\n', debugLevel=1)
                measuredMeanSystemSpeeds.append(systemDistanceTravelled / systemTimeTravelled)

            measuredMeanSystemSpeeds.sort()
            middleMeasuredSystemSpeed = measuredMeanSystemSpeeds[round((SIMULATIONS_PER_CHANGE - 1)/2)]

            printDebug(currentPhaseDistribution, '\t\t\t', middleMeasuredSystemSpeed, '\n', debugLevel=5)

            currentPhaseDistribution = round(currentPhaseDistribution + 1 / resolution,2)
            intersection.setPhaseDistribution(currentPhaseDistribution)






        print ('\n\nALL SIMULATIONS FINISHED')


def runAdjustedSimulation(inputSettings):
    printDebug("inputSettings changed to : ", inputSettings, '\n', debugLevel=1)
    applyInputSettingsNp(inputSettings)

    measuredMeanSystemSpeeds = []


    for j in range(SIMULATIONS_PER_CHANGE):
        seedEntries(SEED + j * 10)

        printDebug("\n\nStarting simulation...", "\n", debugLevel=1)
        # printDebug("meanSpeedsLog: ", meanSystemSpeedLog, "\n", selected=1)
        resetSimulation()
        runSingleSimulation()
        newMeasuredMeanSystemSpeed = systemDistanceTravelled / systemTimeTravelled
        printDebug("measuredSystemSpeed: ", newMeasuredMeanSystemSpeed, '\n', debugLevel=1)
        measuredMeanSystemSpeeds.append(systemDistanceTravelled / systemTimeTravelled)

    simulationResult = 0

    if AVERAGE_NOT_MEDIAN:
        simulationResult = sum(measuredMeanSystemSpeeds) / len(measuredMeanSystemSpeeds)
    else:
        measuredMeanSystemSpeeds.sort()
        middleMeasuredSystemSpeed = measuredMeanSystemSpeeds[round((SIMULATIONS_PER_CHANGE - 1)/2)]
        simulationResult = middleMeasuredSystemSpeed

    global nrSimulationRuns
    nrSimulationRuns += 1

    return simulationResult


def runSimulationAndOptimization():

    searchDirection = 1
    previousResult = 2
    maximumMeanSystemSpeed = 0

    setupSimulation()
    timer.startTime()

    for intersection in intersections:
        for i in range(POINTS_SIMULATED):

            measuredMeanSystemSpeeds = []


            for j in range(SIMULATIONS_PER_CHANGE):
                seedEntries(SEED + j * 10)

                printDebug("\n\nStarting simulation..."
                           "\nPhaseDistribution: ", currentPhaseDistribution, "\n", debugLevel=1)
                # printDebug("meanSpeedsLog: ", meanSystemSpeedLog, "\n", selected=1)
                resetSimulation()
                runSingleSimulation()
                newMeasuredMeanSystemSpeed = systemDistanceTravelled / systemTimeTravelled
                printDebug("measuredSystemSpeed: ", newMeasuredMeanSystemSpeed, '\n', debugLevel=1)
                measuredMeanSystemSpeeds.append(systemDistanceTravelled / systemTimeTravelled)
                timer.lap()

            measuredMeanSystemSpeeds.sort()
            middleMeasuredSystemSpeed = measuredMeanSystemSpeeds[round((SIMULATIONS_PER_CHANGE - 1)/2)]

            if maximumMeanSystemSpeed < middleMeasuredSystemSpeed:
                maximumMeanSystemSpeed = middleMeasuredSystemSpeed
                global bestPhaseDistribution
                bestPhaseDistribution = currentPhaseDistribution

            changeIntersectionTimings2(searchDirection, previousResult, currentPhaseDistribution, middleMeasuredSystemSpeed, intersection)
            previousResult = middleMeasuredSystemSpeed

            intersection.setPhaseDistribution(bestPhaseDistribution) #todo: review this line





        print ('\n\nALL SIMULATIONS FINISHED')
        timer.printAverage(SIMULATIONS_PER_CHANGE * POINTS_SIMULATED)
        # print ('Processing time (per simulation): ' + str(processing_time / SIMULATIONS_PER_CHANGE) + 's')

        # print("\nSimulated time to reach exits:")

        print ("_____________________________________________\n",
               "Best phase distribution: ", bestPhaseDistribution,
               "\nMean system speed: ", maximumMeanSystemSpeed, "m/s",
               "\nMean system speed: ", speedToString(maximumMeanSystemSpeed), "kmh/h",
               "\n_____________________________________________\n",)



class Simulation:

    def runAdjustedSimulation(self, inputSettings):
        return runAdjustedSimulation(inputSettings)

    def setupSimulation(self):
        setupSimulation()

    def getNrIntersections(self):
        return getNrIntersections()

    def getNrSimulationsRun(self):
        return nrSimulationRuns


#runMultipleSimulations()
#runMeasurementSimulations()