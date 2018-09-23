from link import Link
from intersection import Intersection
from car import Car
from scenario import Scenario

import time as time_lib
from timer import Timer
from display import Display




# INITIALIZE CONSTANTS
SPEED_LIMIT = 15 # upper speed limit (m/s)
MAX_DECELERATION = 2.5 # (m/s^2)
ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)
INTERSECTION_SPEED = 8
CAR_LENGTH = 6
MIN_CAR_LENGTH_BETWEEN_CARS = 1.5


SCENARIO_NUMBER = 2
SIMULATIONS_PER_CHANGE = 1
POINTS_SIMULATED = 9
SPEED_IN_KMPERHOUR = True

STOP_DISTANCE_FROM_LINE = 2 # (m) = meters from intersection line to where car stops

# Mean System Speed
CHECK_MEAN_SYSTEM_SPEED_TIMESTEP = 10 * TIME_STEP # (number of timesteps) = only check every X timesteps whether the mean speed has aggregated
MEAN_SYSTEM_SPEED_CONSISTENCY_PERIOD = 60 * 2 # (S) = time during which mean system speed must stay consistent
MEAN_SYSTEM_SPEED_AGGREGATION_PERCENTAGE = 10 # (%) =  current mean system speed must within this percentage of old mean system speed before measuring can begin
MEASURING_PERIOD = 60 * 5 # (s)


# debugging
DEBUG_PRINT_ON = True
DEBUG_PRINT_ONLY_SELECTED = True
DISPLAY_ON = False


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

# needs serious attention
currentPhaseDistribution = 0.1
bestPhaseDistribution = 0.1









def resetSimulation():
    global cars
    global links
    global intersections
    global time
    global systemTimeTravelled
    global systemDistanceTravelled
    global entries
    global measuringStarted

    current_scenario = Scenario.loadScenario(SCENARIO_NUMBER)
    cars = current_scenario['cars']
    links = current_scenario['links']
    intersections = current_scenario['intersections']
    entries = current_scenario['entries']

    time = 0

    if DISPLAY_ON:
        display.setup(links, intersections, 1 / TIME_STEP)


    systemDistanceTravelled = 0
    systemTimeTravelled = 0
    # measuringStarted = 0 # should actually be included, but getting irregular results with different starting times



def speedToString(speed):
    if (SPEED_IN_KMPERHOUR):
        return str(round(speed * 3.6,2)) + str('km/h')
    return str(round(speed,2)) + str('m/s')

def printDebug(*arg, selected = 'NO'):
    if DEBUG_PRINT_ON and ((not DEBUG_PRINT_ONLY_SELECTED) or selected == 'YES'):
        for msg in arg:
            print (msg, end='')


def generateNewCars():
    global entries
    global time

    for entry in entries:
        possibleNewCar = entry.possiblyGenerateCar(time)
        if possibleNewCar:
            cars.append(possibleNewCar)

def runSingleTimestep():
    global cars
    global links
    global intersections
    global time
    global allHasReachedDestination
    global systemTimeTravelled
    global systemDistanceTravelled

    allHasReachedDestination = True

    carsToBeRemoved = []

    generateNewCars2()

    for i in range(len(cars)):

        currentLink, distanceInLink, speed, direction = cars[
            i].getPositionAndSpeed()  # could get this with separate get methods

        printDebug("\n", round(time, 1), "s  ", "\tcar: ", i,
                   "\tcurrentLink: ", currentLink,
                   "\tdistanceInLink: ", round(distanceInLink, 2),
                   " \tspeed: ", speedToString(speed))

        if not cars[i].hasReachedDestination():  # before doing calculations, check if car has already reached destination # should rather delete cars that has reached destination

            allHasReachedDestination = False

            # FINDING CLOSEST OBJECT (can be an intersection or car)

            nextObject_distance, nextObject_speed = findNextObstacle(currentLink, distanceInLink, direction)


            # move
            distanceInLink = cars[i].move2(nextObject_distance, nextObject_speed)



            # routing
            if (distanceInLink > links[currentLink].getDistance()):
                if (links[currentLink].getIntersection(direction) != -1): # change to another link
                    cars[i].addDistanceTravelled(links[currentLink].getDistance())
                    cars[i].changeLink(intersections[links[currentLink].getIntersection(direction)].getLink(direction),
                                       distanceInLink - links[currentLink].getDistance(), direction)
                    # print('Changing to  link', intersections[links[currentLink].getIntersection(direction)].getLink(direction), 'from intersection', links[currentLink].getIntersection(direction), end='')
                else: # car has reached destination
                    cars[i].addDistanceTravelled(distanceInLink)
                    cars[i].changeLink(-1,distanceInLink - links[currentLink].getDistance(), direction)
                    systemDistanceTravelled += cars[i].getDistanceTravelled()
                    systemTimeTravelled += cars[i].getTimeTravelled(time + TIME_STEP) # by this stage, move has already been executed
                    carsToBeRemoved.append(i)

        else:
            printDebug("  \talready arrived")


    carsToBeRemoved.sort(reverse=True)
    for i in carsToBeRemoved:
        cars.pop(i)

    if DISPLAY_ON:
        display.update(cars, time)

    continueSimulation = True
    if round(time,1) % CHECK_MEAN_SYSTEM_SPEED_TIMESTEP == 0:
        continueSimulation = checkMeanSystemSpeed()

    time = round(time + TIME_STEP, 1)

    return continueSimulation

def generateNewCars():
    for entry in entries:
        possibleCar = entry.possiblyGenerateCar(time)
        if possibleCar:
            cars.append(possibleCar)

def generateNewCars2():
    for entry in entries:
        entry.possiblyGenerateCar2(TIME_STEP)

        if entry.hasCarAvailable(): # don't want to spend time on looping through all cars when unnecessary
            distanceTillNextCar = findNextObstacle(entry.getLink(), 0, entry.getDirection())[0]
            if distanceTillNextCar > 0:
                cars.append(entry.possiblyGetCar()) # there will definitely be a car since this is in "if entry.hasCarAvailable"



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

    # todo: initialize with intersection details

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
    runSimulation = True

    while runSimulation:
        runSimulation = runSingleTimestep()

def checkMeanSystemSpeed():
    global measuringStarted
    global systemTimeTravelled
    global systemDistanceTravelled
    global meanSystemSpeedLog

    if measuringStarted:
        if round(time,1) - measuringStarted > MEASURING_PERIOD:
            printDebug("Measuing period is over at ", time, ".\nSimulation stopping...",
                       "\n", selected='YES')
            return False
    elif systemTimeTravelled > 0:
        newMeanSystemSpeed = systemDistanceTravelled/systemTimeTravelled
        meanSystemSpeedLog.append(systemDistanceTravelled/systemTimeTravelled)
        oldMeanSystemSpeed = meanSystemSpeedLog.pop(0)
        if oldMeanSystemSpeed * (1 - MEAN_SYSTEM_SPEED_AGGREGATION_PERCENTAGE / 100) < newMeanSystemSpeed < oldMeanSystemSpeed * (1 + MEAN_SYSTEM_SPEED_AGGREGATION_PERCENTAGE / 100):
            measuringStarted = time
            systemTimeTravelled = 0
            systemDistanceTravelled = 0

            # printDebug("Measuring turned on at ", time, "\nOld mean system speed is ", oldMeanSystemSpeed, "\nNew mean system speed is ", newMeanSystemSpeed, '', selected='YES')

    return True

def changeIntersectionTimings():
    global currentPhaseDistribution
    currentPhaseDistribution += 0.1

    for intersection in intersections:
        intersection.setPhaseDistribution(currentPhaseDistribution)

def measureProcessingTime():
    global previousProcessingTimestamp
    newProcessingTimestamp = time_lib.process_time()
    elapsedProcessingTime = newProcessingTimestamp - previousProcessingTimestamp
    printDebug("Delta processing time: ", elapsedProcessingTime, 's\n', selected='YES')
    previousProcessingTimestamp = newProcessingTimestamp

def startProcessingTimer():
    time_lib.process_time()

def printAverageProcessingTime(divider):
    printDebug("Average processing time: ", elapsedProcessingTime, 's\n', selected='YES')


def runMultipleSimulations():
    maximumMeanSystemSpeed = 0

    timer.startTime()

    for i in range(POINTS_SIMULATED):
        for j in range(SIMULATIONS_PER_CHANGE):

            printDebug("\n\nStarting simulation..."
                       "\nPhaseDistribution: ", currentPhaseDistribution, "\n", selected='YES')
            resetSimulation()
            runSingleSimulation()
            timer.lap()

        measuredMeanSystemSpeed = systemDistanceTravelled / systemTimeTravelled

        if maximumMeanSystemSpeed < measuredMeanSystemSpeed:
            maximumMeanSystemSpeed = measuredMeanSystemSpeed
            global bestPhaseDistribution
            bestPhaseDistribution = currentPhaseDistribution

        changeIntersectionTimings()




    print ('\n\nALL SIMULATIONS FINISHED')
    timer.printAverage(SIMULATIONS_PER_CHANGE * POINTS_SIMULATED)
    # print ('Processing time (per simulation): ' + str(processing_time / SIMULATIONS_PER_CHANGE) + 's')

    # print("\nSimulated time to reach exits:")
    # print(str(round(time, 1)) + "s")

    print ("_____________________________________________\n",
           "Best phase distribution: ", bestPhaseDistribution,
           "\nMean system speed: ", maximumMeanSystemSpeed, "m/s",
           "\nMean system speed: ", speedToString(maximumMeanSystemSpeed), "kmh/h",
           "\n_____________________________________________\n",)


runMultipleSimulations()