from entry import Entry

entry1 = Entry(0, 1, 3600, [1])

for i in range(1000):
    entry1.possiblyGenerateCar2(0.1)
    print (str(entry1.possiblyGetCar()))