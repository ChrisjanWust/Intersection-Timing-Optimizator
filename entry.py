# coding: utf-8

# In[ ]:

from car import Car
import numpy as np

DEBUG_PRINT_ON = False

class Entry:

    def __init__(self, link, direction, vehiclesPerHour, destinations):
        self.link = link
        self.direction = direction
        self.vehiclesPerHour = vehiclesPerHour
        self.destinations = destinations
        self.queue = 0

    def possiblyGenerateCar(self, time):
        if round(time,1) % (3600 / self.vehiclesPerHour) == 0:
            return Car(self.link, self.direction, 10, self.destinations[0])
        return None

    def possiblyGenerateCar2(self, TIME_STEP):
        lamda = TIME_STEP * self.vehiclesPerHour / 3600
        numberOfCarsAdded = np.random.poisson(lamda)
        self.queue += numberOfCarsAdded

        self.printDebug("# generated:", numberOfCarsAdded)

    def possiblyGetCar(self):
        if self.queue > 0:
            self.queue -= 1
            return Car(self.link, self.direction, 10, self.destinations[0])
        return None

    def hasCarAvailable(self):
        if self.queue > 0:
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





