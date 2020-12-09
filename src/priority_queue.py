class PriorityQueue:
    def __init__(self):
        self.items = []
        self.primaries = []
        self.secondaries = []
    
    def put(self, item, primary, secondary):
        self.items.append(item)
        self.primaries.append(primary)
        self.secondaries.append(secondary)

    def getNextIndex(self):
        index = float('inf')
        minA = float('inf')
        minB = float('inf')
        for i in range(len(self.items)):
            if (self.primaries[i] < minA):
                index = i
                minA = self.primaries[i]
                minB = self.secondaries[i]
            elif (self.primaries[i] == minA and self.secondaries[i] < minB):
                index = i
                minA = self.primaries[i]
                minB = self.secondaries[i]
        return int(index)

    def pop(self):
        index = self.getNextIndex()
        toReturn = self.items[index]
        del self.items[index]
        del self.primaries[index]
        del self.secondaries[index]
        return toReturn
    
    def peek(self):
        index = self.getNextIndex()
        return self.items[index]
    
    def topKey(self):
        index = self.getNextIndex()
        return [self.primaries[index], self.secondaries[index]]

    def update(self, item, primary, secondary):
        for i in range(len(self.items)):
            if (self.items[i][0] == item[0] and self.items[i][1] == item[1]):
                self.primaries[i] = primary
                self.secondaries[i] = secondary
                return
    
    def includes(self, item):
        for i in self.items:
            if i[0] == item[0] and i[1] == item[1]:
                return True
        return False
    
    def remove(self, item):
        for i in range(len(self.items)):
            if self.items[i][0] == item[0] and self.items[i][1] == item[1]:
                del self.items[i]
                del self.primaries[i]
                del self.secondaries[i]
                return