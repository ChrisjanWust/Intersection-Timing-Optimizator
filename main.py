from link import Link
from intersection import Intersection
from car import Car
from scenario import Scenario

import copy

import time as time_lib
from display import Display




# INITIALIZE CONSTANTS
SPEED_LIMIT = 15 # upper speed limit (m/s)
MAX_DECELERATION = 2.5 # (m/s^2)
ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)
INTERSECTION_SPEED = 8
CAR_LENGTH = 6
MIN_CAR_LENGTH_BETWEEN_CARS = 2

DEBUG_PRINT_ON = False
DISPLAY_ON = True

SCENARIO_NUMBER = 1
SIMULATIONS_PER_CHANGE = 1
SPEED_IN_KMPERHOUR = True

STOP_DISTANCE_FROM_LINE = 2 # (m) = meters from intersection line to where car stops

# initialize public variables
cars = []
links = []
intersections = []
entries = []
time = 0
systemDistanceTravelled = 0
systemTimeTravelled = 0
display = Display()









def resetSimulation():
    global cars
    global links
    global intersections
    global time
    global systemTimeTravelled
    global systemDistanceTravelled
    global entries

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



def speedToString(speed):
    if (SPEED_IN_KMPERHOUR):
        return str(round(speed * 3.6,2)) + str('km/h')
    return str(round(speed,2)) + str('m/s')

def printDebug(*arg):
    if DEBUG_PRINT_ON:
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

            # initializing variables with intersection values before looking if there is another car between the current car (about to be moved) and the intersection
            nextObject_distance = links[currentLink].getDistance() - distanceInLink - STOP_DISTANCE_FROM_LINE
            nextObject_speed = INTERSECTION_SPEED

            if ((intersections[links[currentLink].getIntersection(direction)].getStatus(time, direction) == "RED") ):
                nextObject_speed = 0

            for j in range(len(cars)): # het hier 'n gedeelte kode wat eksponentieel meer word volgens hoeveelheid motors in die netwerk !! Sal moet optimaliseer deur array te rangskik of so iets
                if (not cars[j].hasReachedDestination()):
                    # printDebug('helooooooooooooooooooooooooooooooo\n\n\n')

                    comparisonCar_currentLink, comparisonCar_distanceInLink, comparisonCar_speed, comparisonCar_direction = cars[j].getPositionAndSpeed()
                    if (
                            comparisonCar_currentLink == currentLink
                            and comparisonCar_direction == direction
                            and comparisonCar_distanceInLink > distanceInLink
                            and comparisonCar_distanceInLink - distanceInLink - CAR_LENGTH * MIN_CAR_LENGTH_BETWEEN_CARS < nextObject_distance
                    ):
                        # printDebug('\n\nDOES THIS EXECUTE?\n\n')
                        nextObject_distance = comparisonCar_distanceInLink - distanceInLink - CAR_LENGTH * MIN_CAR_LENGTH_BETWEEN_CARS
                        nextObject_speed = comparisonCar_speed

                        # printDebug('\tCar', j, nextObject_distance, 'meters away')
                        # printDebug('\tCar', j, nextObject_distance, 'meters away')

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


    for i in carsToBeRemoved:
        cars.pop(i)

    if DISPLAY_ON:
        display.update(cars, time)

    time += TIME_STEP

    return allHasReachedDestination

def generateNewCars():
    for entry in entries:
        possibleCar = entry.possiblyGenerateCar(time)
        if possibleCar:
            cars.append(possibleCar)

def generateNewCars2():
    for entry in entries:
        entry.possiblyGenerateCar2(TIME_STEP)
        distanceTillNextCar = findNextObstacle(entry.getLink(), 0)[0]
        if distanceTillNextCar > 0:
            possibleCar = entry.possiblyGetCar()
            if possibleCar:
                cars.append(possibleCar)



def findNextObstacle(linkIndex, distanceInLink):
    objectDistance = links[linkIndex].getDistance() - distanceInLink
    objectSpeed = 0

    # todo: initialize with intersection details

    for car in cars:
        carLinkIndex, carDistanceInLink, carSpeed, carDirection = car.getPositionAndSpeed()

        # statements can be combined, but keep for now for readability
        if carLinkIndex == linkIndex:
            if distanceInLink < carDistanceInLink:
                if carDistanceInLink - distanceInLink < objectDistance:
                    objectDistance = carDistanceInLink - distanceInLink - CAR_LENGTH * MIN_CAR_LENGTH_BETWEEN_CARS
                    objectSpeed = carSpeed

    return objectDistance, objectSpeed


def runSingleSimulation():
    allHasReachedDestination = False

    while (not allHasReachedDestination):
        allHasReachedDestination = runSingleTimestep()  # kort cleanup




def runMultipleSimulations():
    time_lib.process_time()

    for i in range(SIMULATIONS_PER_CHANGE):
        resetSimulation()
        runSingleSimulation()

    processing_time = time_lib.process_time()

    print ('\n\nProcessing time (total): ' + str(processing_time) + 's')
    print ('Processing time (per simulation): ' + str(processing_time / SIMULATIONS_PER_CHANGE) + 's')

    print("\nSimulated time to reach exits:")
    print(str(round(time, 1)) + "s")

    print("\nSystem distance travelled:", str(round(systemDistanceTravelled,2)) + "m")
    print("System time travelled:", str(round(systemTimeTravelled, 2)) + 's')
    print("Mean system speed:", speedToString(systemDistanceTravelled/systemTimeTravelled))


runMultipleSimulations()
