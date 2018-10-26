# coding: utf-8

# In[ ]:

DEBUG_PRINT_ON = False

DEFAULT_PHASE_DISTRIBUTION = 0.5
DEFAULT_PERIOD = 60

YELLOW_TIME = 3
RED_TIME = 2

class Intersection:
    
    # need to add dynamic different timing cycles
    
    def __init__(self, links, phaseDistribution = DEFAULT_PHASE_DISTRIBUTION, period = DEFAULT_PERIOD, offset=0):
        self.links = links
        self.phaseDistribution = phaseDistribution
        self.period = period
        self.offset = offset

    #def getStatusOld(self, time, direction):
    #    if (time % 60 > self.timings[direction][0] and time % 60 < self.timings[direction][1]):
    #        return "GREEN"
    #    else:
    #        return "RED"

    def getStatus(self, time, direction):
        timeWithOffset = time - self.offset
        timePerUnit = (timeWithOffset % self.period) / self.period
        self.printDebug("Intersection details at ", timeWithOffset, "s", "\tDirection:", direction, "\tpu:", timePerUnit)

        effectiveRedTimePerUnit = (YELLOW_TIME / 2 + RED_TIME) / self.period
        if direction % 2 == 0 and timePerUnit < self.phaseDistribution - effectiveRedTimePerUnit:
            self.printDebug('\tTRUE\n')
            return "GREEN"
        elif direction % 2 == 1 and self.phaseDistribution < timePerUnit < 1 - effectiveRedTimePerUnit:
            self.printDebug('\tTRUE\n')
            return "GREEN"
        self.printDebug('\tFALSE\n')
        return "RED"
        
    def getLink (self, direction):
        return self.links[direction]

    def setPhaseDistribution(self, phaseDistribution):
        self.phaseDistribution = phaseDistribution

    def setPeriod(self, period):
        self.period = period

    def setOffset(self, offset):
        self.offset = offset


    def printDebug(self, *arg):
        if DEBUG_PRINT_ON:
            for msg in arg:
                print(msg, end='')
