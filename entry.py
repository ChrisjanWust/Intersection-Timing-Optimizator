# coding: utf-8

# In[ ]:

from car import Car
import numpy as np

AGGRESSIVENESS_STD_DEVIATION = 0.04

DEBUG_PRINT_ON = False

class Entry:

    def __init__(self, link, direction, vehiclesPerHour, destinations):
        self.link = link
        self.direction = direction
        self.vehiclesPerHour = vehiclesPerHour
        self.destinations = destinations
        self.queue = 0
        self.carQueue = []


    def seed(self, seed_number):
        np.random.seed(seed_number)


    def possiblyGenerateCarOrg(self, time):
        if round(time,1) % (3600 / self.vehiclesPerHour) == 0:
            return Car(self.link, self.direction, 10, self.destinations[0])
        return None

    def possiblyGenerateCarOrg2(self, TIME_STEP):
        lamda = TIME_STEP * self.vehiclesPerHour / 3600
        numberOfCarsAdded = np.random.poisson(lamda)
        self.queue += numberOfCarsAdded

        self.printDebug("# generated:", numberOfCarsAdded)

    def possiblyGenerateCarDeterministic(self, TIME_STEP, time):
        if round(time,1) % (3600 / self.vehiclesPerHour) == 0:
            self.carQueue.append(Car(self.link, self.direction, 10, self.destinations[0], time))
            self.printDebug("Car generated")


    def possiblyGenerateCar(self, TIME_STEP, time):
        lamda = TIME_STEP * self.vehiclesPerHour / 3600
        numberOfCarsAdded = np.random.poisson(lamda)

        for i in range(numberOfCarsAdded):
            self.carQueue.append(Car(self.link, self.direction, 10, self.destinations[0], timeEntered=time, aggressiveness=np.random.normal(1, AGGRESSIVENESS_STD_DEVIATION)))

        self.printDebug("# generated:", numberOfCarsAdded)

    def possiblyGetCar(self):
        if self.carQueue:
            return self.carQueue.pop(0)
        return None

    def hasCarAvailable(self):
        if self.carQueue:
            return True
        return False


    def getLink(self):
        return self.link

    def getDirection(self):
        return self.direction

    def printDebug(self, *arg):
        if DEBUG_PRINT_ON:
            for msg in arg:
                print(msg, end='')





