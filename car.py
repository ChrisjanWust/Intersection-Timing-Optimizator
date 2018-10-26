
# coding: utf-8

# In[ ]:

import numpy as np

SPEED_LIMIT = 15 # upper speed limit (m/s)
TARGET_SPEED = 12
MAX_DECELERATION = 2.5 # (m/s^2)
MAX_ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)
FOLLOWING_TIME = 2 # (s)
MIN_SPEED_IN_SINGLE_LINK = 0.6 # (m/s)

DEBUG_PRINT_ON = False

class Car:
    links = []

    def __init__(self, currentLinkIndex, direction, speed, destinationLinkIndex, timeEntered = 0.0, distanceInLink = 0.0, aggressiveness = 1):
        self.currentLinkIndex = currentLinkIndex
        self.destinationLinkIndex = destinationLinkIndex
        #self.currentLink = Link(currentLink[0])
        self.distanceInLink = distanceInLink
        self.speed = speed
        self.direction = direction
        self.distanceTravelled = 0
        self.timeEntered = timeEntered
        self.aggressiveness = aggressiveness # normal is 1. Greater than one is more aggressive. Shouldn't be more than 1.4
        self.timeLinkChanged = timeEntered


    def printDebug(self, *arg):
        if DEBUG_PRINT_ON:
            for msg in arg:
                print(msg, end='')


    def setTimeEntered(self, timeEntered):
        self.timeEntered = timeEntered

        
    def move2(self, distanceTillNextObject, speedOfNextObject):
        oldSpeed = self.speed

        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject)

        # self.printDebug('\tmove2: dx: ', round(distanceTillNextObject,2), 'm  \tv2: ',  round(speedOfNextObject,2), 'm/s')
        if (distanceTillNextObject >= START_BREAKING_DISTANCE or self.speed < speedOfNextObject): # accelerate
            if(self.speed < SPEED_LIMIT):
                self.speed = self.speed + MAX_ACCELERATION * TIME_STEP
        else: # try to decelerate (might have stopped already)
            if (self.speed != 0):
                # deceleration = ((START_BREAKING_DISTANCE - distanceTillNextObject) * (
                #        START_BREAKING_DISTANCE - distanceTillNextObject) / START_BREAKING_DISTANCE / START_BREAKING_DISTANCE * 1.3 + self.speed / SPEED_LIMIT * self.speed / SPEED_LIMIT * 1.43) * MAX_DECELERATION * TIME_STEP

                # new deceleration based on first principles / equation of motion
                if distanceTillNextObject != 0:
                    deceleration = - (speedOfNextObject * speedOfNextObject - self.speed * self.speed) / ( 2 * (distanceTillNextObject))
                else:
                    deceleration = MAX_DECELERATION
                # self.printDebug('  \tdecelerating at ', round(deceleration,2), 'm/s2')

                # if (deceleration > MAX_DECELERATION): # can remove this test later?
                    # deceleration = MAX_DECELERATION # deceleration should actually be limited, but not for now
                    # self.printDebug('  \tdecelerating above max')
                self.speed = self.speed - deceleration * TIME_STEP
                if self.speed < 0:
                    self.speed = 0

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink

    def move2withAggressiveness(self, distanceTillNextObject, speedOfNextObject):
        oldSpeed = self.speed

        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject)

        # self.printDebug('\tmove2: dx: ', round(distanceTillNextObject,2), 'm  \tv2: ',  round(speedOfNextObject,2), 'm/s')
        if (distanceTillNextObject >= START_BREAKING_DISTANCE / self.aggressiveness or self.speed < speedOfNextObject): # accelerate
            if(self.speed < SPEED_LIMIT):
                self.speed = self.speed + MAX_ACCELERATION * self.aggressiveness * TIME_STEP
        else: # try to decelerate (might have stopped already)
            if (self.speed != 0):
                # deceleration = ((START_BREAKING_DISTANCE - distanceTillNextObject) * (
                #        START_BREAKING_DISTANCE - distanceTillNextObject) / START_BREAKING_DISTANCE / START_BREAKING_DISTANCE * 1.3 + self.speed / SPEED_LIMIT * self.speed / SPEED_LIMIT * 1.43) * MAX_DECELERATION * TIME_STEP

                # new deceleration based on first principles / equation of motion
                if distanceTillNextObject <= 0:
                    deceleration = MAX_DECELERATION * 10 # hacky
                else:
                    deceleration = - (speedOfNextObject * speedOfNextObject - self.speed * self.speed) / ( 2 * (distanceTillNextObject))
                # self.printDebug('  \tdecelerating at ', round(deceleration,2), 'm/s2')

                # if (deceleration > MAX_DECELERATION): # can remove this test later?
                    # deceleration = MAX_DECELERATION # deceleration should actually be limited, but not for now
                    # self.printDebug('  \tdecelerating above max')
                self.speed = self.speed - deceleration * TIME_STEP
                if self.speed < 0:
                    self.speed = 0

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink

    def move2a(self, distanceTillNextObject, speedOfNextObject):
        oldSpeed = self.speed

        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject)

        # self.printDebug('\tmove2: dx: ', round(distanceTillNextObject,2), 'm  \tv2: ',  round(speedOfNextObject,2), 'm/s')
        if (distanceTillNextObject >= START_BREAKING_DISTANCE or self.speed < speedOfNextObject): # accelerate

            referenceDistance = distanceTillNextObject - speedOfNextObject * FOLLOWING_TIME

            K_acc = 4
            acceleration = K_acc * (speedOfNextObject * speedOfNextObject - self.speed * self.speed) / (
                        2 * (referenceDistance))



            if acceleration < 0:
                acceleration = 0.1 * MAX_ACCELERATION
            elif acceleration > MAX_ACCELERATION:
                acceleration = MAX_ACCELERATION

            self.speed = self.speed + K_acc * acceleration * TIME_STEP

            if self.speed > SPEED_LIMIT:
                self.speed = SPEED_LIMIT

        else: # try to decelerate (might have stopped already)
            if (self.speed != 0):
                # deceleration = ((START_BREAKING_DISTANCE - distanceTillNextObject) * (
                #        START_BREAKING_DISTANCE - distanceTillNextObject) / START_BREAKING_DISTANCE / START_BREAKING_DISTANCE * 1.3 + self.speed / SPEED_LIMIT * self.speed / SPEED_LIMIT * 1.43) * MAX_DECELERATION * TIME_STEP

                # new deceleration based on first principles / equation of motion
                referenceDistance = distanceTillNextObject - speedOfNextObject * FOLLOWING_TIME

                if referenceDistance <= 0:
                    referenceDistance = 0.001

                deceleration = - (speedOfNextObject * speedOfNextObject - self.speed * self.speed) / (2 * (referenceDistance))
                # self.printDebug('  \tdecelerating at ', round(deceleration,2), 'm/s2')

                if (deceleration > MAX_DECELERATION): # can remove this test later?
                    deceleration = MAX_DECELERATION # deceleration should actually be limited, but not for now
                    # self.printDebug('  \tdecelerating above max')

                self.speed = self.speed - deceleration * TIME_STEP
                if self.speed < 0:
                    self.speed = 0

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink

    def move3(self, distanceTillNextObject, v_next):
        oldSpeed = self.speed


        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject)

        K_distance = 0.1
        K_acceleration = 0.2
        d_ref_nom = FOLLOWING_TIME * v_next
        delta_d = distanceTillNextObject - d_ref_nom

        V_ref_nom = SPEED_LIMIT
        V_ref = V_ref_nom + K_distance * delta_d

        a = K_acceleration * (V_ref - oldSpeed)
        self.printDebug("a:", a)
        self.speed = oldSpeed + a * TIME_STEP


        # limiting
        if self.speed < 0.4: # limit moet hoer as nul - gaan neig na 0, maar nooit 0 wees nie
            self.speed = 0
        if self.speed > SPEED_LIMIT:
            self.speed = SPEED_LIMIT

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink





    def move4(self, distanceTillNextObject, speedOfNextObject):
        oldSpeed = self.speed


        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject, '\n')

        K_d = 0.3
        K_v = 0.8 / FOLLOWING_TIME
        d_ref_nom = FOLLOWING_TIME * speedOfNextObject
        delta_d = distanceTillNextObject - d_ref_nom

        v_ref = SPEED_LIMIT
        if distanceTillNextObject < START_BREAKING_DISTANCE:
            v_ref = speedOfNextObject

        a = K_v * (pow(v_ref,2) - pow(oldSpeed, 2)) + K_d * delta_d

        self.printDebug('a: ', a)

        if a > MAX_ACCELERATION:
            a = MAX_ACCELERATION
        elif a < - MAX_DECELERATION:
            a = - MAX_DECELERATION

        self.speed = oldSpeed + a * TIME_STEP


        # limiting
        if self.speed < 0: # limit moet hoer as nul - gaan neig na 0, maar nooit 0 wees nie
            self.speed = 0
        if self.speed > SPEED_LIMIT:
            self.speed = SPEED_LIMIT

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink


    def move5(self, d_next, v_next):
        global MAX_DECELERATION
        #MAX_DECELERATION += 1

        oldSpeed = self.speed

        if (d_next < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', d_next)

        # self.printDebug('\tmove2: dx: ', round(distanceTillNextObject,2), 'm  \tv2: ',  round(speedOfNextObject,2), 'm/s')
        if (d_next >= START_BREAKING_DISTANCE or self.speed < v_next):  # accelerate

            d_ref = v_next * FOLLOWING_TIME
            d_delta = d_ref - d_next

            v_ref = SPEED_LIMIT
            if d_next < START_BREAKING_DISTANCE:
                v_ref = v_next

            v_delta = SPEED_LIMIT - v_ref

            K_v = 1
            K_d = 0.1
            acceleration = K_v * v_delta + K_d * d_delta

            if acceleration < 0.1 * MAX_ACCELERATION:
                acceleration = 0.1 * MAX_ACCELERATION
            elif acceleration > MAX_ACCELERATION:
                acceleration = MAX_ACCELERATION

            self.speed = self.speed + acceleration * TIME_STEP

            self.printDebug("acc:", acceleration)

            if self.speed > SPEED_LIMIT:
                self.speed = SPEED_LIMIT

        else:  # try to decelerate (might have stopped already)
            if (self.speed != 0):
                # deceleration = ((START_BREAKING_DISTANCE - distanceTillNextObject) * (
                #        START_BREAKING_DISTANCE - distanceTillNextObject) / START_BREAKING_DISTANCE / START_BREAKING_DISTANCE * 1.3 + self.speed / SPEED_LIMIT * self.speed / SPEED_LIMIT * 1.43) * MAX_DECELERATION * TIME_STEP

                # new deceleration based on first principles / equation of motion
                referenceDistance = d_next - v_next * FOLLOWING_TIME

                if referenceDistance <= 0:
                    referenceDistance = 0.001

                deceleration = - (v_next * v_next - self.speed * self.speed) / (
                            2 * (referenceDistance))
                # self.printDebug('  \tdecelerating at ', round(deceleration,2), 'm/s2')

                if (deceleration > MAX_DECELERATION):  # can remove this test later?
                    deceleration = MAX_DECELERATION  # deceleration should actually be limited, but not for now
                    # self.printDebug('  \tdecelerating above max')

                self.speed = self.speed - deceleration * TIME_STEP

                self.printDebug("dec:", deceleration)

                if self.speed < 0:
                    self.speed = 0

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink


    def setPositionAndSpeed(self, currentLinkIndex, distanceInLink, speed, direction):
        self.currentLinkIndex = currentLinkIndex
        self.distanceInLink = distanceInLink
        self.speed = speed
        self.direction = direction


    def changeLink(self, newLinkIndex, newDistanceInLink, newDirection, time=None):
        self.currentLinkIndex = newLinkIndex
        self.distanceInLink = newDistanceInLink
        self.direction = newDirection
        self.timeLinkChanged = time

    def checkConstraintsOk (self, time):
        if time - self.timeLinkChanged > 60:
            if self.distanceInLink / (time - self.timeLinkChanged + 0.0000001) < MIN_SPEED_IN_SINGLE_LINK:
                return False

        return True

    def addDistanceTravelled(self, distance):
        self.distanceTravelled += distance

    def getPositionAndSpeed(self):
        return (self.currentLinkIndex, self.distanceInLink, self.speed, self.direction)

    def setSpeed(self, speed):
        self.speed = speed


    def hasReachedDestination(self, currentLinkDistance):
        if self.currentLinkIndex == self.destinationLinkIndex:
            if self.distanceInLink > currentLinkDistance:
                return True
        return False

    def hasReachedDestination(self):
        if (self.currentLinkIndex == -1):
            return True
        return False
    
    def assignLinks(newLinks):
        Car.links = newLinks

    def getTimeTravelled(self, timeExited):
        return timeExited - self.timeEntered

    def getDistanceTravelled(self):
        return self.distanceTravelled


