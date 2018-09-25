from listGenerator import ListGenerator

listGenerator = ListGenerator

list2d = listGenerator.twoD(3, 4, [])

print('\n\n', list2d)

list2d[1][2].append(9)

print (list2d)



list3d = listGenerator.threeD(7, 4, 0)

print('\n\n', list3d)

list3d[1][1].append(4)

print('\n\n', list3d)
