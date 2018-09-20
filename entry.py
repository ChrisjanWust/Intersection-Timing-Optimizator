# coding: utf-8

# In[ ]:

from car import Car
import numpy as np


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
        self.queue += np.random.poisson(TIME_STEP * 3600 / self.vehiclesPerHour)

    def possiblyGetCar(self):
        if self.queue > 0:
            self.queue -= 1
            return Car(self.link, self.direction, 10, self.destinations[0])
        return None

    def getLink(self):
        return self.link




