import math
import re
from priority_queue import PriorityQueue
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty

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
        self.foundPath = True

        rospy.init_node('path_planner', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber('/project/pose', Point, self.updatePosition)
        rospy.Subscriber('/project/task_planner', Point, self.updateGoal)
        rospy.Subscriber('/map', OccupancyGrid, self.updateGrid)
        rospy.Subscriber('/project/path_planner_request', Empty, self.givePoint)

        rospy.Publisher('/project/path_planner', Point, queue_size=1)

        self.computeShortestPath()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def updateVertex(self, x, y):
        if self.grid[y][x]['g'] != self.grid[y][x]['rhs'] and self.queue.includes((y, x)):
            key = self.calculateKey(x, y)
            self.queue.update((y, x), key[0], key[1])
        elif self.grid[y][x]['g'] != self.grid[y][x]['rhs'] and not self.queue.includes((y, x)):
            key = self.calculateKey(x, y)
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
            while self.keyComp(self.queue.topKey(), self.calculateKey(self.startX, self.startY)) == -1 or self.grid[self.startY][self.startX]['rhs'] > self.grid[self.startY][self.startX]['g']:
                item = self.queue.peek()
                k_old = self.queue.topKey()
                k_new = self.calculateKey(item[1], item[0])
                if self.keyComp(k_old, k_new) == -1:
                    self.queue.update(item, k_new[0], k_old[1])
                elif self.grid[item[0]][item[1]]['g'] > self.grid[item[0]][item[1]]['rhs']:
                    self.grid[item[0]][item[1]]['g'] = self.grid[item[0]][item[1]]['rhs']
                    self.queue.remove(item)
                    for y,x in self.neighbors(item[0], item[1], False):
                        if not (y == self.goalY and x == self.goalX):
                            newRhs = self.grid[item[0]][item[1]]['g'] + self.cost(item[1], item[0], x, y)
                            if newRhs < self.grid[y][x]['rhs']:
                                self.grid[y][x]['rhs'] = newRhs
                                self.grid[y][x]['next'] = (item[0], item[1])
                        self.updateVertex(x, y)
                else:
                    g_old = self.grid[item[0]][item[1]]['g']
                    self.grid[item[0]][item[1]]['g'] = float('inf')
                    for y,x in self.neighbors(item[0], item[1], True):
                        if self.grid[y][x]['rhs'] == self.cost(item[1], item[0], x, y) + g_old:
                            if not (y == self.goalY and x == self.goalX):
                                minRhs = float('inf')
                                for yn, xn in self.neighbors(y, x, False):
                                    if minRhs > self.cost(xn, yn, x, y) + self.grid[yn][xn]['g']:
                                        minRhs = self.cost(xn, yn, x, y) + self.grid[yn][xn]['g']
                                        self.grid[y][x]['next'] = (yn, xn)
                                self.grid[y][x]['rhs'] = minRhs
                        self.updateVertex(x, y)
            self.foundPath = True
        except Exception:
            self.foundPath = False
            print("Cannot find a path")

    def cost(self, x1, y1, x2, y2):
        if self.grid[y1][x1]['open'] < 1 or self.grid[y2][x2]['open'] < 1:
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

    def updatePosition(self, point):
        self.startX, self.startY = self.pointToXY(point)
    
    def updateGoal(self, point):
        self.goalX, self.goalY = self.pointToXY(point)

    def updateGrid(self, msg):
        if len(self.grid == 0):
            file = open("../complete_devon.yaml")
            for line in file:
                m = re.search("(.+?): (.*)", line)
                if (m):
                    if m.group(1) == "resolution":
                        self.resolution = float(m.group(2))
                    elif m.group(1) == "origin":
                        m = re.search("\[(-?\d+\.\d+), (-?\d+\.\d+), -?\d+\.\d+\]", m.group(2))
                        self.offset = (m.group(1), m.group(2))
            file.close()
            for y in msg.data:
                index = len(grid)
                self.grid.append([])
                for x in msg.data[y]:
                    toAdd = {'open': int(x ), 'rhs': float('inf'), 'g': float('inf'), 'next': None}
                    self.grid[index].append(toAdd)
            self.grid[self.goalY][self.goalX]['rhs'] = 0
            self.queue.put((self.goalY, self.goalX), math.hypot(self.goalX - self.startX, self.goalY - self.startY), 0)
            self.expandGridWalls()
            return
        changes = []
        for y in msg.data:
            for x in msg.data[y]:
                clear = msg.data[y][x] < 20
                if self.grid[y][x]["open"] != int(clear):
                    changes.append((x, y, self.grid[y][x]["open"]))
                    self.grid[y][x] = int(clear)
        self.expandGridWalls()
        acted = False
        for point in changes:
            if self.grid[point[1]][point[0]]["open"] != point[3]:
                if not acted:
                    self.km += math.hypot(self.startY - self.oldY, self.startX - self.oldX)
                    self.oldX = self.startX
                    self.oldY = self.startY
                acted = True
                for y,x in self.neighbors(point[1], point[0], False):
                    if self.grid[point[1]][point[0]]["open"] == 1:
                        if not (y == self.goalY and x == self.goalX):
                            newRhs = min(self.grid[y][x]["rhs"], self.grid[point[1]][point[0]]["g"] + self.cost(x, y, point[0], point[1]))
                            if (newRhs < self.grid[y][x]["rhs"]):
                                self.grid[y][x]["rhs"] = newRhs
                                self.grid[y][x]["next"] = (point[1], point[0])
                    elif self.grid[y][x]["next"][0] == point[1] and self.grid[y][x]["next"][1] == point[0]:
                        if not (y == self.goalY and x == self.goalX):
                            for y2,x2 in self.neighbors(y, x, False):
                                newRhs = self.grid[y2][x2]['g'] + self.cost(x2, y2, x, y)
                                if newRhs < self.grid[y][x]['rhs']:
                                    self.grid[y][x]['rhs'] = newRhs
                                    self.grid[y][x]['next'] = (y2, x2)
                    self.updateVertex(x, y)
        if (acted):
            computeShortestPath()
    
    def expandGridWalls(self):
        wallList = []
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                if (self.grid[y][x]["open"] == 0):
                    wallList.append((y, x))
                if (self.grid[y][x]["open"] == -1):
                    self.grid[y][x]["open"] == 1
        for point in wallList:
            for i in range(11):
                for j in range(11):
                    point = (point[0] + i - 5, point[1] + j - 5)
                    if point[0] >= len(self.grid) or point[0] < 0:
                        continue
                    if point[1] >= len(self.grid[0]) or point[1] < 0:
                        continue
                    self.grid[point[0]][point[1]]["open"] = -1
    
    def givePoint(self):
        if self.foundPath:
            nextPoint = self.grid[self.startY][self.startX]["next"]
            self.pointPublisher.publish(self.xyToPoint(nextPoint[1], nextPoint[0]))
    
    def xyToPoint(self, x, y):
        toReturn = Point()
        toReturn.x = x * self.resolution + self.offset[0]
        toReturn.y = y * self.resolution + self.offset[1]
        

    def pointToXY(self, point):
        return int((point[0] - self.offset[0]) / self.resolution), int((point[1] - self.offset[1]) / self.resolution)

    def shutdown(self):
        print("Shutting down")

if __name__ == "__main__":
    PathPlanner()