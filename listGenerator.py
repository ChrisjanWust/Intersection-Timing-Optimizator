class ListGenerator:
    def __init__(self):
        self.uselessVariable = 0
        # print()

    def twoD(self, firstLevelSize,secondLevelSize,defaultValue):
        list = [[defaultValue]]

        for i in range(0, firstLevelSize):
            # print('\n', i, ': ', end='')

            if i != 0:
                list.append([defaultValue])

            for j in range(1, secondLevelSize):
                # print (j, ', ', end='')
                list[i].append(defaultValue)

        return list

    def threeD(self, firstLevelSize,secondLevelSize, thirdLevelSize, defaultValueGiven = False, defaultValue=None):
        # this code isn't optimized, but oh well

        if defaultValueGiven == True:
            list = [[[defaultValue]]]

            for i in range(firstLevelSize):
                # print('\n-- ', i, ' --', end='')
                if i != 0:
                    list.append([[defaultValue]])

                for j in range(secondLevelSize):
                    # print ('\n', j, ': ', end='')
                    if j != 0: # todo: daar is 'n beter manier, verander net for loop
                        list[i].append([defaultValue])

                    for k in range(thirdLevelSize):
                        # print (k, ', ', end='')
                        if k != 0: # todo: daar is 'n beter manier, verander net for loop
                            list[i][j].append(defaultValue)

            return list

        if defaultValueGiven == False:
            list = [[[]]]

            for i in range(firstLevelSize):
                # print('\n-- ', i, ' --', end='')
                if i != 0:
                    list.append([[]])

                for j in range(secondLevelSize):
                    # print('\n', j, ': ', end='')
                    if j != 0:  # todo: daar is 'n beter manier, verander net for loop
                        list[i].append([])

                    for k in range(thirdLevelSize):
                        # print(k, ', ', end='')
                        if k != 0:  # todo: daar is 'n beter manier, verander net for loop
                            list[i][j].append()

            return list

