
# coding: utf-8

# In[ ]:

from link import Link


SPEED_LIMIT = 15 # upper speed limit (m/s)
MAX_DECELERATION = 2.5 # (m/s^2)
ACCELERATION = 0.9 # (m/s^2)
TIME_STEP = 0.1 # (s)
START_BREAKING_DISTANCE = 40 # (m)



class Car:
    links = []
    
    def __init__(self, currentLinkIndex, direction, speed, destinationLinkIndex):
        self.currentLinkIndex = currentLinkIndex
        self.destinationLinkIndex = destinationLinkIndex
        #self.currentLink = Link(currentLink[0])
        self.distanceInLink = 0
        self.speed = speed
        self.direction = direction

        
    def move(self, nextStop):
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


        if (distanceTillNextObject <= START_BREAKING_DISTANCE | speedOfNextObject >= self.speed): # accelerate
            self.speed = self.speed + ACCELERATION * TIME_STEP
        else: # try to decelerate (might have stopped already)
            if (self.speed != 0):
                deceleration = ((START_BREAKING_DISTANCE - distanceTillNextObject) * (
                        START_BREAKING_DISTANCE - distanceTillNextObject) / START_BREAKING_DISTANCE / START_BREAKING_DISTANCE * 1.3 + self.speed / SPEED_LIMIT * self.speed / SPEED_LIMIT * 1.43) * MAX_DECELERATION * TIME_STEP
                if (deceleration > MAX_DECELERATION):
                    deceleration = MAX_DECELERATION
                self.speed = self.speed - deceleration
                if self.speed < 0:
                    self.speed = 0

        self.distanceInLink += TIME_STEP * (self.speed + oldSpeed) / 2

        return self.distanceInLink

    def setPositionAndSpeed(self, currentLinkIndex, distanceInLink, speed, direction):
        self.currentLinkIndex = currentLinkIndex
        self.distanceInLink = distanceInLink
        self.speed = speed
        self.direction = direction
        
    def getPositionAndSpeed(self):
        return (self.currentLinkIndex, self.distanceInLink, self.speed, self.direction)
    
    def hasReachedDestination(self, currentLinkDistance):
        if self.currentLinkIndex == self.destinationLinkIndex:
            if self.distanceInLink > currentLinkDistance:
                return True
        return False
    
    def assignLinks(newLinks):
        Car.links = newLinks
           
        

