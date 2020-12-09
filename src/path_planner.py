#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty

import math
import re
import os

from priority_queue import PriorityQueue


class PathPlanner():

    def __init__(self, map_file_path):
        self.map_file_path = map_file_path

        #A bunch of initial variables
        self.grid = []
        self.queue = PriorityQueue()
        self.goalX = -1
        self.goalY = -1
        self.startX = -1
        self.startY = -1
        self.oldX = 0
        self.oldY = 0
        self.km = 0
        self.foundPath = False

        # idk if this is correct
        self.offset = (0,0)
        self.resolution = 0.5

        #ROS setup stuff
        rospy.init_node('path_planner', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('init path planner with map file ' + self.map_file_path)

        #We need so much data
        #Position
        rospy.Subscriber('/project/pose', Point, self.updatePosition)
        #Goal
        rospy.Subscriber('/project/task_planner', Point, self.updateGoal)
        #Map
        rospy.Subscriber('/map', OccupancyGrid, self.updateGrid)
        #Trigger to send out plan
        rospy.Subscriber('/project/next_point_in_path', Empty, self.givePoint)

        rospy.Subscriber('/project/replan_path', Empty, self.replan)
        rospy.Subscriber('/project/path_planner_dump', Empty, self.writeMap)

        #Sending out data to the navigation layer
        self.pointPublisher = rospy.Publisher('/project/path_planner', Point, queue_size=1)

        #Compute the shortest path, then wait for messages
        self.computeShortestPath()
        
        rospy.spin()

    #Update vertex keys and have the queue react accordingly
    def updateVertex(self, x, y):
        #If the node has a shorter way to go than before, let the queue know
        if self.grid[y][x]['g'] != self.grid[y][x]['rhs'] and self.queue.includes((y, x)):
            key = self.calculateKey(x, y)
            self.queue.update((y, x), key[0], key[1])
        #If the node has a longer way to go than before and hasn't been marked, put it on the queue to be processed
        elif self.grid[y][x]['g'] != self.grid[y][x]['rhs'] and not self.queue.includes((y, x)):
            key = self.calculateKey(x, y)
            self.queue.put((y, x), key[0], key[1])
        #If everything's looking good, mark it off the queue
        elif self.grid[y][x]['g'] == self.grid[y][x]['rhs'] and self.queue.includes((y, x)):
            self.queue.remove((y, x))

    #Comparison based on primary and secondary keys
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

    #Main function
    def computeShortestPath(self):
        #Throws an error when there is no path
        try:
            #Run until we've passed the start in our max reach and the start needs to be measured
            while self.keyComp(self.queue.topKey(), self.calculateKey(self.startX, self.startY)) == -1 or self.grid[self.startY][self.startX]['rhs'] > self.grid[self.startY][self.startX]['g']:
                #Get the top item
                item = self.queue.peek()
                #Get the top key and check what it should be
                k_old = self.queue.topKey()
                k_new = self.calculateKey(item[1], item[0])
                #If the key increased, let the queue know
                if self.keyComp(k_old, k_new) == -1:
                    self.queue.update(item, k_new[0], k_old[1])
                #If the node needs to have its measurements lower
                elif self.grid[item[0]][item[1]]['g'] > self.grid[item[0]][item[1]]['rhs']:
                    #Measure the node
                    self.grid[item[0]][item[1]]['g'] = self.grid[item[0]][item[1]]['rhs']
                    #Mart the node as done
                    self.queue.remove(item)
                    #Expand nearby nodes
                    for y,x in self.neighbors(item[0], item[1], False):
                        #If this isn't the goal
                        if not (y == self.goalY and x == self.goalX):
                            #Estimate the cost of going to this cell
                            newRhs = self.grid[item[0]][item[1]]['g'] + self.cost(item[1], item[0], x, y)
                            #If that's good, tell that cell to go to this cell
                            if newRhs < self.grid[y][x]['rhs']:
                                self.grid[y][x]['rhs'] = newRhs
                                self.grid[y][x]['next'] = (item[0], item[1])
                        #Save changes
                        self.updateVertex(x, y)
                #Assume this node sucks and needs to be assumed the worst node
                else:
                    #Save the old g-value and set the new one to infinity
                    g_old = self.grid[item[0]][item[1]]['g']
                    self.grid[item[0]][item[1]]['g'] = float('inf')
                    #For all the neighbors, including self
                    for y,x in self.neighbors(item[0], item[1], True):
                        #If it was going through this node
                        if self.grid[y][x]['rhs'] == self.cost(item[1], item[0], x, y) + g_old:
                            #If it's not the goal
                            if not (y == self.goalY and x == self.goalX):
                                #Find the best possible path from here
                                minRhs = float('inf')
                                for yn, xn in self.neighbors(y, x, False):
                                    if minRhs > self.cost(xn, yn, x, y) + self.grid[yn][xn]['g']:
                                        minRhs = self.cost(xn, yn, x, y) + self.grid[yn][xn]['g']
                                        self.grid[y][x]['next'] = (yn, xn)
                                self.grid[y][x]['rhs'] = minRhs
                        #Mark in the queue
                        self.updateVertex(x, y)
            #If we got here, we found a path!
            print("Found a path!")
            self.foundPath = True
        except Exception as e:
            #Handle path
            self.foundPath = False
            rospy.logerr("Cannot find a path")
            rospy.logerr(e)

    #Calculate the cost of an edge. Infinite if impossible, else the cartesian distance.
    def cost(self, x1, y1, x2, y2):
        if self.grid[y1][x1]['open'] < 1 or self.grid[y2][x2]['open'] < 1:
            return float('inf')
        else:
            return math.hypot(y1 - y2, x1 - x2)

    #Calculate the key of a node
    def calculateKey(self, x, y):
        return [min(self.grid[y][x]['g'], self.grid[y][x]['rhs']) + math.hypot(x - self.startX, y - self.startY) + self.km, min(self.grid[y][x]['g'], self.grid[y][x]['rhs'])]

    #Enumerate all a node's neighbors, optionally including itself
    def neighbors(self, y, x, includeSelf):
        for i in range(3):
            for j in range(3):
                toYield = (y + i - 1, x + j - 1)
                #Don't give nonexistent results
                if toYield[0] >= len(self.grid) or toYield[0] < 0:
                    continue
                if toYield[1] >= len(self.grid[0]) or toYield[1] < 0:
                    continue
                #Only give self if asked
                if toYield[0] == y and toYield[1] == x and not includeSelf:
                    continue
                yield toYield[0], toYield[1]

    #Set position
    def updatePosition(self, point):
        # TODO: check if this is correct. pointToXY -> error because offset is a string?
        #rospy.loginfo("Setting position")
        #rospy.loginfo(point)
        self.startX, self.startY = self.pointToXY(point) # point.x, point.y 
        #rospy.loginfo(self.startX)
        #rospy.loginfo(self.startY)

    
    #Set goal
    def updateGoal(self, point):
        rospy.loginfo('path planner goal recieved')
        rospy.loginfo(point)
        self.goalX, self.goalY = self.pointToXY(point)
        rospy.loginfo("Updated goal:")
        rospy.loginfo(self.goalX)
        rospy.loginfo(self.goalY)
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                self.grid[y][x]["rhs"] = float("inf")
                self.grid[y][x]["g"] = float("inf")
                self.grid[y][x]["next"] = None
        self.foundPath = False
        self.queue = PriorityQueue()
        self.queue.put((self.goalY, self.goalX), math.hypot(self.goalX - self.startX, self.goalY - self.startY), 0)
        if len(self.grid) > 0:
            self.grid[self.goalY][self.goalX]["rhs"] = 0
            self.computeShortestPath()
            self.givePoint(None)

    #Update grid from the map
    def updateGrid(self, msg):
        #If there is no grid
        if len(self.grid) == 0:
            #Check the map settings
            # TODO: pass this in via launch file
            file = open(self.map_file_path)
            for line in file:
                #Read the line
                m = re.search("(.+?): (.*)", line)
                if (m):
                    #Get resolution
                    # if m.group(1) == "resolution":
                    #     self.resolution = float(m.group(2))
                    #Get origin
                    if m.group(1) == "origin":
                        m = re.search("\[(-?\d+\.\d+), (-?\d+\.\d+), -?\d+\.\d+\]", m.group(2))
                        # TODO: check this cast to float
                        self.offset = (float(m.group(1)), float(m.group(2)))
                        print('setting offset to ', self.offset)
            file.close()

            # handle map metadata
            # self.resolution = msg.info.resolution
            self.grid = [[None] * msg.info.width for _ in range(msg.info.height)]
            
            #For each row, mark a row
            for i in range(msg.info.height):
                for j in range(msg.info.width):
                    #For each item in the row, add the corresponding row
                    # TODO: check this, because it throughs an error (x is an int)
                    cell = msg.data[i * msg.info.width + j]
                    toAdd = {'open': int(cell < 20), 'rhs': float('inf'), 'g': float('inf'), 'next': None}
                    self.grid[i][j] = toAdd

            #Set the goal RHS to 0 and put it on the queue
            self.grid[self.goalY][self.goalX]['rhs'] = 0
            self.queue.put((self.goalY, self.goalX), math.hypot(self.goalX - self.startX, self.goalY - self.startY), 0)
            #Make our concept of the walls bigger so we don't hit them
            self.expandGridWalls()
            return

        #Keep track of any changes
        changes = []
        #Make changes
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                #For each item in the row, add the corresponding row
                # TODO: check this, because it throughs an error (x is an int)
                clear = msg.data[i * msg.info.width + j] < 20
                if self.grid[i][j]["open"] != int(clear):
                    changes.append((j, i, self.grid[i][j]["open"]))
                    self.grid[i][j] = int(clear)

        #Do the wall thing again
        self.expandGridWalls()
        #Keep track of if anything changes
        acted = False
        for point in changes:
            #If we actually changed after considering the phantom walls
            if self.grid[point[1]][point[0]]["open"] != point[3]:
                #If this is the first time this cycle, make the changes required
                if not acted:
                    #I honestly don't know what's up with this. What even is KM
                    self.km += math.hypot(self.startY - self.oldY, self.startX - self.oldX)
                    self.oldX = self.startX
                    self.oldY = self.startY
                    acted = True
                #For all the neighbors of the changed point
                for y,x in self.neighbors(point[1], point[0], False):
                    #If the node got open
                    if self.grid[point[1]][point[0]]["open"] == 1:
                        #If the neighbor isn't the node
                        if not (y == self.goalY and x == self.goalX):
                            #Check if we can make the RHS better with this new info
                            newRhs = min(self.grid[y][x]["rhs"], self.grid[point[1]][point[0]]["g"] + self.cost(x, y, point[0], point[1]))
                            #If we can, do it
                            if (newRhs < self.grid[y][x]["rhs"]):
                                self.grid[y][x]["rhs"] = newRhs
                                self.grid[y][x]["next"] = (point[1], point[0])
                    #If the node got closed and the neighbor was going here
                    elif self.grid[y][x]["next"][0] == point[1] and self.grid[y][x]["next"][1] == point[0]:
                        #If the neighbor isn't the goal
                        if not (y == self.goalY and x == self.goalX):
                            #Check what neighbors you can go to and go to the best one
                            for y2,x2 in self.neighbors(y, x, False):
                                newRhs = self.grid[y2][x2]['g'] + self.cost(x2, y2, x, y)
                                if newRhs < self.grid[y][x]['rhs']:
                                    self.grid[y][x]['rhs'] = newRhs
                                    self.grid[y][x]['next'] = (y2, x2)
                    self.updateVertex(x, y)
        #Update if needed
        if (acted):
            computeShortestPath()
    
    #Make our estimate of the walls bigger
    def expandGridWalls(self):
        #Keep track of the walls
        wallList = []
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                if (self.grid[y][x]["open"] == 0):
                    wallList.append((y, x))
                #Set all the phantom walls to open
                if (self.grid[y][x]["open"] == -1):
                    self.grid[y][x]["open"] == 1
        #Make all the cells close to walls closed off
        for point in wallList:
            for i in range(11):
                for j in range(11):
                    considering = (point[0] + i - 5, point[1] + j - 5)
                    if considering[0] >= len(self.grid) or considering[0] < 0:
                        continue
                    if considering[1] >= len(self.grid[0]) or considering[1] < 0:
                        continue
                    self.grid[considering[0]][considering[1]]["open"] = -1
    
    #Respond to a point request
    def givePoint(self, msg):
        if self.foundPath:
            print(self.foundPath)
            print("I think I found a path")
            nextPoint = self.grid[self.startY][self.startX]["next"]
            if (nextPoint == None):
                rospy.loginfo('No path found!')
                return
            else:

                self.pointPublisher.publish(self.xyToPoint(nextPoint[1], nextPoint[0]))
        else:
            rospy.loginfo('no path found!')
    
    def replan(self, msg):
        self.computeShortestPath()
        self.givePoint(None)

    #Convert coordinates to a point
    def xyToPoint(self, x, y):
        print("Converting point", x, y, "Which is not the starting point", self.startX, self.startY)
        toReturn = Point(0, 0, 0)
        toReturn.x = x * self.resolution + self.offset[0]
        toReturn.y = y * self.resolution + self.offset[1]
        return toReturn
        
    #Convert a point to coordinates
    def pointToXY(self, point):
        return int(round((point.x - self.offset[0]) / self.resolution)), int(round((point.y - self.offset[1]) / self.resolution))

    #Shut down
    def shutdown(self):
        print("Shutting down")
    
    def writeMap(self, msg):
        print("Dumping map")
        #Change this to make the dump work
        #I'm sure there's a way to do this relative but it didn't work when I tried it
        file = open(os.path.dirname(os.path.realpath(__file__)) + "/../map.pgm", 'wb')
        file.write("P5\n768 704\n255\n")
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                if y == self.goalY and x == self.goalX:
                    file.write(b"\x55")
                elif y == self.startY and x == self.startX:
                    file.write(b"\xAA")
                else:
                    file.write(b"\x00" if self.grid[y][x]["open"] < 1 else b"\xFE")
        file.close()

        file = open(os.path.dirname(os.path.realpath(__file__)) + "/../g.pgm", 'wb')
        file.write("P5\n768 704\n255\n")
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                if (self.grid[y][x]["g"] > 255):
                    file.write(b"\x00")
                else:
                    file.write(bytes([int(255 - self.grid[y][x]["g"])]))
        file.close()

        file = open(os.path.dirname(os.path.realpath(__file__)) + "/../rhs.pgm", 'wb')
        file.write("P5\n768 704\n255\n")
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                if (self.grid[y][x]["rhs"] > 510):
                    file.write(b"\x00")
                else:
                    file.write(bytes([int(255 - self.grid[y][x]["rhs"] / 2)]))
        file.close()

        file = open(os.path.dirname(os.path.realpath(__file__)) + "/../next.pgm", 'wb')
        file.write("P5\n768 704\n255\n")
        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                file.write(b"\xFE" if self.grid[y][x]["next"] == None else b"\x00")
        file.close()

#Run
if __name__ == "__main__":
    import sys

    PathPlanner(sys.argv[1])