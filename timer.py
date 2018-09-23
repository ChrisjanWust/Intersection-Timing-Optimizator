import time as time

DEBUG_PRINT_ON = True
DEBUG_PRINT_ONLY_SELECTED = False

class Timer:

    def __init__(self):
        self.startupTime = 0
        self.previousTimestamp = 0

    def startTime(self):
        self.startupTime = time.process_time()
        self.previousTimestamp = self.startupTime

    def lap(self):
        newProcessingTimestamp = time.process_time()
        elapsedProcessingTime = newProcessingTimestamp - self.previousTimestamp
        self.printDebug("Delta processing time: ", elapsedProcessingTime, 's\n', selected='YES')
        self.previousTimestamp = newProcessingTimestamp

    def printAverage(self, nrRuns):
        currentTimestamp = time.process_time()
        averageTime = (currentTimestamp - self.startupTime) / nrRuns
        self.printDebug("Average processing time: ", averageTime, 's\n', selected='YES')

    def printDebug(*arg, selected='NO'):
        if DEBUG_PRINT_ON and ((not DEBUG_PRINT_ONLY_SELECTED) or selected == 'YES'):
            for msg in arg:
                print(msg, end='')