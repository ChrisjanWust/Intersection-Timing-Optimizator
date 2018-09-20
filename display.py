import pygame

SCALE = 2
WINDOW_HEIGHT = 800
WINDOW_WIDTH = 1400
CAR_LENGTH = 5
CAR_WIDTH = 2
ROAD_COLOR = (150,150,150)
CAR_COLOR = (0, 0, 230)
ROAD_WIDTH = 6

TIME_TEXT_SIZE = 48

FIRST_INTERSECTION_INDEX = 0
FIRST_INTERSECTION_X = 200
FIRST_INTERSECTION_Y = 200


#           DIRECTIONS
#
#               0
#               ↑
#          3 ←    → 1
#               ↓
#               2


class Display:


    def __init__(self):
        self.intersections = []
        self.links = []

        self.window = pygame.display.set_mode((1200, 1000))
        self.fps = 10
        self.clock = pygame.time.Clock()

        # metadata about links and intersections to help with mapping
        self.linksCoordinates = [[]] * len(
            self.links)  # stores coordinates which will be used for calculating the coordinates of cars in the link
        self.linksDirections = [None] * len(
            self.links)  # similar to linksCoordinates, but stores the direction of the link (0 = vertical, 1 = horizontal) # could merge these two link metadata arrays, but oh well
        self.unexploredIntersections = []  # deprecating
        # self.font = pygame.font.SysFont("bahnschrift", TIME_TEXT_SIZE)



    def setup(self, links, intersections, fps = 10):
        self.intersections = intersections
        self.links = links

        self.window = pygame.display.set_mode((1200, 1000))
        self.fps = fps
        self.clock = pygame.time.Clock()

        # metadata about links and intersections to help with mapping
        self.linksCoordinates = [[]] * len(
            self.links)  # stores coordinates which will be used for calculating the coordinates of cars in the link
        self.linksDirections = [None] * len(
            self.links)  # similar to linksCoordinates, but stores the direction of the link (0 = vertical, 1 = horizontal) # could merge these two link metadata arrays, but oh well
        self.unexploredIntersections = []  # deprecating



        pygame.init()



        self.mapNetwork()




    def drawCars(self, cars):
        for car in cars:
            linkIndex, distanceInLink, speed, direction = car.getPositionAndSpeed()
            linkDistance = self.links[linkIndex].getDistance()
            linkCoordinates = self.linksCoordinates[linkIndex]
            frontOfCar = self.directionalAdd(direction, linkCoordinates, distanceInLink - linkDistance / 2)

            # change coordinate for specific lane
            directionToLane = (direction - 1 + 4) % 4
            frontOfCar = self.directionalAdd(directionToLane, frontOfCar, ROAD_WIDTH / 4)


            backOfCar = self.directionalAdd(direction, frontOfCar, - CAR_LENGTH)
            pygame.draw.line(self.window, CAR_COLOR, self.scaleList(frontOfCar), self.scaleList(backOfCar), SCALE * CAR_WIDTH)

    def drawNetwork(self):
        for linkIndex in range(len(self.links)):
            point1 = self.directionalAdd(self.linksDirections[linkIndex], self.linksCoordinates[linkIndex], self.links[linkIndex].getDistance()/2)
            point2 = self.directionalAdd(self.oppositeDirection(self.linksDirections[linkIndex]), self.linksCoordinates[linkIndex], self.links[linkIndex].getDistance()/2)
            # print("Drawing line from", point1, "to", point2)
            pygame.draw.line(self.window, ROAD_COLOR, self.scaleList(point1), self.scaleList(point2), ROAD_WIDTH * SCALE)


    def drawTime(self, time):
        text = self.font.render(str(time), True, (200, 200, 200))
        self.window.blit(text, (WINDOW_HEIGHT - TIME_TEXT_SIZE - 30, WINDOW_WIDTH - 150))


    def exploreIntersection(self, intersectionIndex, intersectionCoordinates):
        for direction in range(4):
            linkIndex = self.intersections[intersectionIndex].getLink(direction)
            if not self.linksCoordinates[linkIndex ]:
                self.linksCoordinates[linkIndex] = self.directionalAdd(direction,
                                                                       intersectionCoordinates,
                                                                       ROAD_WIDTH/2 + self.links[linkIndex].getDistance()/2)
                self.linksDirections[linkIndex] = direction % 2
                # link is unexplored. Possibly need to explore the intersection at the end.
                nextIntersectionIndex = self.links[linkIndex].getIntersection(direction)
                if nextIntersectionIndex != -1:
                    self.exploreIntersection(nextIntersectionIndex,
                                             self.directionalAdd(direction,
                                                                intersectionCoordinates,
                                                                ROAD_WIDTH + self.links[linkIndex].getDistance()))






    # deprecating
    def getNextUnexploredLink(self, intersectionIndex, direction):
        direction +=1
        if (direction != 4):
            if self.linksCoordinates[self.intersections[intersectionIndex].getLink(direction)]:
                self.getNextUnexploredLink(intersectionIndex, direction + 1)
            else:
                return self.intersections[intersectionIndex].getLink(direction)
        else:
            # no unexplored link in the intersection
            return -1




    def directionalAdd(self, direction, startingCoordinates, add): # desperately need to test this method
        multiplierMatrix = [[0,1,0,-1],[-1,0,1,0]]
        return [startingCoordinates[0] + multiplierMatrix[0][direction] * add, startingCoordinates[1] + multiplierMatrix[1][direction] * add]


    def mapNetwork(self):
        self.exploreIntersection(FIRST_INTERSECTION_INDEX, [FIRST_INTERSECTION_X, FIRST_INTERSECTION_Y])
        print(self.linksCoordinates)


    def oppositeDirection (self, direction):
        if direction == 0:
            return 2
        elif direction == 1:
            return 3
        elif direction == 2:
            return 0
        elif direction == 3:
            return 1

    def scaleList(self, originalValue):
        return [i * SCALE for i in originalValue]


    def update(self, cars, time):
        self.window.fill((0,0,0))

        self.drawNetwork()
        self.drawCars(cars)
        # self.drawTime(time)

        self.clock.tick(self.fps)
        pygame.display.update()


    def runTillExit(self):
        running = True
        while running:
            # for loop through the event queue
            for event in pygame.event.get():
                # Check for KEYDOWN event; KEYDOWN is a constant defined in pygame.locals, which we imported earlier
                if event.type == pygame.KEYDOWN:
                    # If the Esc key has been pressed set running to false to exit the main loop
                    if event.key == pygame.K_ESCAPE:
                        running = False
                # Check for QUIT event; if QUIT, set running to false
                elif event.type == pygame.QUIT:
                    running = False

