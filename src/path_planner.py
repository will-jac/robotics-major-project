import math
from priority_queue import PriorityQueue
import rospy
import tf2_ros

class PathPlanner():

    def __init__(self):
        self.grid = []
        self.queue = PriorityQueue()
        self.goalX = 0
        self.goalY = 0
        self.startX = 0
        self.startY = 0
        self.oldX = 0
        self.oldY = 0
        self.km = 0

        tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)
        
        self.initializeGrid()
        self.updatePosition()

        self.computeShortestPath()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.updatePosition()
            self.computeShortestPath()
            self.updateGrid()
            rate.sleep()

        #Debug stuff
        nextX = self.startX
        nextY = self.startY
        while self.grid[nextY][nextX]['next'] != None:
            print(nextX, nextY)
            nextX, nextY = self.grid[nextY][nextX]['next'][1], self.grid[nextY][nextX]['next'][0]
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                print("x:", j, "y:", i, "rhs:", self.grid[i][j]["rhs"], "g:", self.grid[i][j]["g"], "next:", self.grid[i][j]["next"])


    def initializeGrid(self):
        file = open("map.txt")
        for line in file:
            index = len(self.grid)
            self.grid.append([])
            arr = line.split()
            for cell in arr:
                toAdd = {'open': int(cell), 'rhs': float('inf'), 'g': float('inf'), 'next': None}
                self.grid[index].append(toAdd)
        file.close()
        self.grid[self.goalY][self.goalX]['rhs'] = 0
        self.queue.put((self.goalY, self.goalX), math.hypot(self.goalX - self.startX, self.goalY - self.startY), 0)

    def updateVertex(self, x, y):
        if self.grid[y][x]['g'] != self.grid[y][x]['rhs'] and self.queue.includes((y, x)):
            key = calculateKey(x, y)
            self.queue.update((y, x), key[0], key[1])
        elif self.grid[y][x]['g'] != self.grid[y][x]['rhs'] and not self.queue.includes((y, x)):
            key = calculateKey(x, y)
            self.queue.put((y, x), key[0], key[1])
        elif self.grid[y][x]['g'] == self.grid[y][x]['rhs'] and self.queue.includes((y, x)):
            self.queue.remove((y, x))

    def keyComp(self, A, B):
        if A[0] > B[0]:
            return 1
        elif A[0] < B[0]:
            return -1
        elif A[1] > B[1]:
            return 1
        elif A[1] < B[1]:
            return -1
        return 0

    def computeShortestPath(self):
        try:
            while keyComp(self.queue.topKey(), calculateKey(self.startX, self.startY)) == -1 or self.grid[self.startY][self.startX]['rhs'] > self.grid[self.startY][self.startX]['g']:
                item = self.queue.peek()
                k_old = self.queue.topKey()
                k_new = calculateKey(item[1], item[0])
                if keyComp(k_old, k_new) == -1:
                    self.queue.update(item, k_new[0], k_old[1])
                elif self.grid[item[0]][item[1]]['g'] > self.grid[item[0]][item[1]]['rhs']:
                    self.grid[item[0]][item[1]]['g'] = self.grid[item[0]][item[1]]['rhs']
                    self.queue.remove(item)
                    for y,x in neighbors(item[0], item[1], False):
                        if not (y == self.goalY and x == self.goalX):
                            newRhs = self.grid[item[0]][item[1]]['g'] + cost(item[1], item[0], x, y)
                            if newRhs < self.grid[y][x]['rhs']:
                                self.grid[y][x]['rhs'] = newRhs
                                self.grid[y][x]['next'] = (item[0], item[1])
                        updateVertex(x, y)
                else:
                    g_old = self.grid[item[0]][item[1]]['g']
                    self.grid[item[0]][item[1]]['g'] = float('inf')
                    for y,x in neighbors(item[0], item[1], True):
                        if self.grid[y][x]['rhs'] == cost(item[1], item[0], x, y) + g_old:
                            if not (y == self.goalY and x == self.goalX):
                                minRhs = float('inf')
                                for yn, xn in neighbors(y, x, False):
                                    if minRhs > cost(xn, yn, x, y) + self.grid[yn][xn]['g']:
                                        minRhs = cost(xn, yn, x, y) + self.grid[yn][xn]['g']
                                        self.grid[y][x]['next'] = (yn, xn)
                                self.grid[y][x]['rhs'] = minRhs
                        updateVertex(x, y)
        except Exception:
            print("Cannot find a path")

    def cost(self, x1, y1, x2, y2):
        if self.grid[y1][x1]['open'] == 0 or self.grid[y2][x2]['open'] == 0:
            return float('inf')
        else:
            return math.hypot(y1 - y2, x1 - x2)

    def calculateKey(self, x, y):
        return [min(self.grid[y][x]['g'], self.grid[y][x]['rhs']) + math.hypot(x - self.startX, y - self.startY) + self.km, min(self.grid[y][x]['g'], self.grid[y][x]['rhs'])]

    def neighbors(self, y, x, includeSelf):
        for i in range(3):
            for j in range(3):
                toYield = (y + i - 1, x + j - 1)
                if toYield[0] >= len(self.grid) or toYield[0] < 0:
                    continue
                if toYield[1] >= len(self.grid[0]) or toYield[1] < 0:
                    continue
                if toYield[0] == y and toYield[1] == x and not includeSelf:
                    continue
                yield toYield[0], toYield[1]

    def updatePosition(self):
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            if (trans.transform is None):
                continue
            self.startX = trans.transform.translation.x
            self.startY = trans.transform.translation.y
        except:
            pass

    def updateGrid(self):
        #TODO: This
        print("NYI")
    
    def expandGridWalls(self):
        wallList = []
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                if (self.grid[y][x]["open"] == 0):
                    wallList.append((y, x))
        for point in wallList:
            for i in range(3):
                for j in range(3):
                    point = (point[0] + i - 1, point[1] + j - 1)
                    if point[0] >= len(self.grid) or point[0] < 0:
                        continue
                    if point[1] >= len(self.grid[0]) or point[1] < 0:
                        continue
                    self.grid[point[0]][point[1]]["open"] = 0