from display import Display
from scenario import Scenario
from car import Car

import time

SCENARIO_NUMBER = 1


current_scenario = Scenario.loadScenario(1)
cars = current_scenario['cars']
links = current_scenario['links']
intersections = current_scenario['intersections']

display = Display(links, intersections)
display.setup(links, intersections)

display.update([Car(1, 1, 6, 3, distanceInLink=1), Car(0, 0, 9, 2, distanceInLink=1)])

display.update([Car(1, 1, 6, 3, distanceInLink=4), Car(0, 0, 9, 2, distanceInLink=1)])

display.runTillExit()




time.sleep(2)