from link import Link
from intersection import Intersection
from car import Car
from scenario import Scenario

import copy

import time as time_lib


# INITIALIZE CONSTANTS
SPEED_LIMIT = 15 # upper speed limit (m/s)
MAX_DECELERATION = 2.5 # (m/s^2)
ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)

DEBUG_PRINT_ON = True
SCENARIO_NUMBER = 1

# initialize public variables
cars = None
links = None
intersections = None
time = 0










def reset_simulation():
    global cars
    global links
    global intersections
    global time

    current_scenario = Scenario.load_scenario(SCENARIO_NUMBER)
    cars = current_scenario['cars']
    links = current_scenario['links']
    intersections = current_scenario['intersections']
    time = 0



def print_debug(*arg):
    if DEBUG_PRINT_ON:
        for msg in arg:
            print (msg, end='')



def single_time_step():
    global cars
    global links
    global intersections
    global time
    global allHasReachedDestination

    allHasReachedDestination = True

    for i in range(len(cars)):

        currentLink, distanceInLink, speed, direction = cars[
            i].getPositionAndSpeed()  # could get this with separate get methods

        print_debug(  "\n",
                      round(time, 1), "s",
                      "\tcar: ", i,
                      "\tcurrentLink: ", currentLink,
                      "\tdistanceInLink: ", round(distanceInLink, 2),
                      "\tspeed: ", round(speed, 2)
                    )

        if not cars[i].hasReachedDestination(links[currentLink].getDistance()):  # before doing calculations, check if car has already reached destination

            allHasReachedDestination = False

            # if not far enough in, don't mind
            distanceTillIntersection = links[currentLink].getDistance() - distanceInLink

            newSpeed = speed

            if (links[currentLink].getIntersection(direction) == -1
                    or intersections[links[currentLink].getIntersection(direction)].getStatus(time,
                                                                                              direction) == "GREEN"
                    or distanceTillIntersection > START_BREAKING_DISTANCE):
                # next intersection is green

                if speed < SPEED_LIMIT:  # increase speed if below upper speed limit
                    newSpeed = speed + ACCELERATION * TIME_STEP  # todo: check if speed is above speed limit, limit again

                distanceInLink += TIME_STEP * (speed + newSpeed) / 2
                if distanceInLink > links[currentLink].getDistance():
                    if links[currentLink].getIntersection(direction) == -1:  # check if its a exit
                        print_debug("\n\nCar ", i, " has just reached its destination\n\n")
                    else:
                        distanceInLink -= links[currentLink].getDistance()
                        currentLink = intersections[links[currentLink].getIntersection(direction)].getLink(direction)

            else:
                # next intersection is red

                if speed > 0:
                    newSpeed = speed - MAX_DECELERATION * TIME_STEP  # needs to be much smarter
                    deceleration = ((START_BREAKING_DISTANCE - distanceTillIntersection) * (
                            START_BREAKING_DISTANCE - distanceTillIntersection) / START_BREAKING_DISTANCE / START_BREAKING_DISTANCE * 1.3 + speed / SPEED_LIMIT * speed / SPEED_LIMIT * 1.43) * MAX_DECELERATION * TIME_STEP
                    if (deceleration > MAX_DECELERATION):
                        deceleration = MAX_DECELERATION
                    newSpeed = speed - deceleration
                    if newSpeed < 0:
                        newSpeed = 0

                distanceInLink += TIME_STEP * (speed + newSpeed) / 2

            cars[i].setPositionAndSpeed(currentLink, distanceInLink, newSpeed, direction)

        else:
            print_debug("\talready arrived")

    time += TIME_STEP

    return allHasReachedDestination









# run
reset_simulation()

allHasReachedDestination = False

time_lib.process_time()

while (not allHasReachedDestination):
    allHasReachedDestination = single_time_step() # kort cleanup

processing_time = time_lib.process_time()
print ('\n\nProcessing time: ' + str(processing_time) + 's')

print("\n\nSimulated time to reach exits:")
print(str(round(time, 1)) + "s")

