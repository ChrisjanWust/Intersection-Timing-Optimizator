from car import Car

car1 = Car(0,0,8,2)


            # [d_next, v_next, v_self]
dataPoints = [[10,0,3],[39,6,6],[3,2,1],[-3,2,1]]
for dataPoint in dataPoints:
    print(dataPoint)
    car1.setSpeed(dataPoint[2])
    car1.move4(dataPoint[0], dataPoint[1])
    print('\n')