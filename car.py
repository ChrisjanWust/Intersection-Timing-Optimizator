
# coding: utf-8

# In[ ]:



SPEED_LIMIT = 15 # upper speed limit (m/s)
TARGET_SPEED = 12
MAX_DECELERATION = 2.5 # (m/s^2)
ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)
AVERAGE_FOLLOWING_DISTANCE = 2 # (s)

DEBUG_PRINT_ON = False

class Car:
    links = []
    
    def __init__(self, currentLinkIndex, direction, speed, destinationLinkIndex, distanceInLink = 0.0, timeEntered = 0.0):
        self.currentLinkIndex = currentLinkIndex
        self.destinationLinkIndex = destinationLinkIndex
        #self.currentLink = Link(currentLink[0])
        self.distanceInLink = distanceInLink
        self.speed = speed
        self.direction = direction
        self.distanceTravelled = 0
        self.timeEntered = timeEntered


    def printDebug(self, *arg):
        if DEBUG_PRINT_ON:
            for msg in arg:
                print(msg, end='')

    def setTimeEntered(self, timeEntered):
        self.timeEntered = timeEntered

        
    def move(self, nextStop): #deprecating
        # need a lot of modification; for starters, should consider accelleration
        self.distanceInLink += 0.1 * self.speed
        
        # routing
        if self.distanceInLink > Car.links[self.currentLinkIndex].distance:
            
            # over simplistic. Needs pathfinder
            if self.direction == 0 or self.direction == 3:
                self.currentLinkIndex = Car.links[self.currentLinkIndex].intersections[0][self.direction]
            else:
                self.currentLinkIndex = Car.links[self.currentLinkIndex].intersections[1][self.direction]
            
            # over simplistic. Needs to take intersection into account (distances / stop / start)
            self.distanceInLink = self.distanceInLink - Car.links[self.currentLinkIndex].distance
            
            if self.currentLinkIndex == -1: # car exited, reached destination
                return True
        
        return False

    def move2(self, distanceTillNextObject, speedOfNextObject):
        oldSpeed = self.speed

        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject)

        # self.printDebug('\tmove2: dx: ', round(distanceTillNextObject,2), 'm  \tv2: ',  round(speedOfNextObject,2), 'm/s')
        if (distanceTillNextObject >= START_BREAKING_DISTANCE or self.speed < speedOfNextObject): # accelerate
            if(self.speed < SPEED_LIMIT):
                self.speed = self.speed + ACCELERATION * TIME_STEP
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

    def move3(self, distanceTillNextObject, speedOfNextObject):
        oldSpeed = self.speed


        if (distanceTillNextObject < 0):
            self.printDebug('distanceTillNextObject smaller than 0!! It is:', distanceTillNextObject)

        K_distance = 2
        K_acceleration = 1
        d_ref_nom = AVERAGE_FOLLOWING_DISTANCE * oldSpeed
        delta_d = distanceTillNextObject - d_ref_nom

        V_ref_nom = TARGET_SPEED
        V_ref = V_ref_nom + K_distance * delta_d
        a = K_acceleration * (V_ref - oldSpeed)
        self.speed = oldSpeed + a * TIME_STEP


        # limiting
        if self.speed < 0: # limit moet hoer as nul - gaan neig na 0, maar nooit 0 wees nie
            self.speed = 0
        if self.speed > SPEED_LIMIT:
            self.speed = SPEED_LIMIT

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink

    def setPositionAndSpeed(self, currentLinkIndex, distanceInLink, speed, direction):
        self.currentLinkIndex = currentLinkIndex
        self.distanceInLink = distanceInLink
        self.speed = speed
        self.direction = direction


    def changeLink(self, newLinkIndex, newDistanceInLink, newDirection):
        self.currentLinkIndex = newLinkIndex
        self.distanceInLink = newDistanceInLink
        self.direction = newDirection

    def addDistanceTravelled(self, distance):
        self.distanceTravelled += distance

    def getPositionAndSpeed(self):
        return (self.currentLinkIndex, self.distanceInLink, self.speed, self.direction)

    def getCurrentLinkIndex(self):
        return self.currentLinkIndex

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

    def getDirection(self):
        return self.direction

    def getTimeTravelled(self, timeExited):
        return timeExited - self.timeEntered

    def getDistanceTravelled(self):
        return self.distanceTravelled


