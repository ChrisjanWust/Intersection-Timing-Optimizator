class LinkMetadata:

    def __init__(self):
        self.coordinates = []

    def setCoordinatesAndExplored(self, coordinates):
        self.coordinates = coordinates

    def hasBeenExplored(self):
        if self.coordinates:
            return False
        return True

    def getCoordinates(self):
        return self.coordinates