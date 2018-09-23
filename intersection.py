# coding: utf-8

# In[ ]:

DEBUG_PRINT_ON = False

class Intersection:
    
    # need to add dynamic different timing cycles
    
    def __init__(self, links, phaseDistribution=0.5, period=40):
        self.links = links
        self.phaseDistribution = phaseDistribution
        self.period = period

    #def getStatusOld(self, time, direction):
    #    if (time % 60 > self.timings[direction][0] and time % 60 < self.timings[direction][1]):
    #        return "GREEN"
    #    else:
    #        return "RED"

    def getStatus(self, time, direction):
        perUnit = (time % self.period) / self.period
        self.printDebug("Intersection details at ", time, "s", "\tDirection:", direction, "\tpu:", perUnit)
        if direction % 2 == 0 and perUnit < self.phaseDistribution:
            self.printDebug('\tTRUE\n')
            return "GREEN"
        elif direction % 2 == 1 and perUnit > self.phaseDistribution:
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


    def printDebug(self, *arg):
        if DEBUG_PRINT_ON:
            for msg in arg:
                print(msg, end='')
