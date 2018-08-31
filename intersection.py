# coding: utf-8

# In[ ]:


class Intersection:
    
    # need to add dynamic different timing cycles
    
    def __init__(self, links, timings):
        self.links = links
        self.timings = timings

    def getStatus(self, time, direction):
        if (time % 60 > self.timings[direction][0] and time % 60 < self.timings[direction][1]):
            return "GREEN"
        else:
            return "RED"
        
    def getLink (self, direction):
        return self.links[direction]
