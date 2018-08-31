
# coding: utf-8

# In[ ]:


class Link:
    
    def __init__(self, distance, intersections):
        self.intersections = intersections
        self.distance = distance
        
    def getIntersection(self, direction):
        return self.intersections[direction]
        
    def getDistance(self):
        return self.distance


