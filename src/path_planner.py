import math
from priority_queue import PriorityQueue
grid = []
queue = PriorityQueue()
goalX = 0
goalY = 0
startX = 4
startY = 2
km = 0

def initialize():
    file = open("map.txt")
    for line in file:
        index = len(grid)
        grid.append([])
        arr = line.split()
        for cell in arr:
            toAdd = {'open': int(cell), 'rhs': float('inf'), 'g': float('inf'), 'next': None}
            grid[index].append(toAdd)
    grid[goalY][goalX]['rhs'] = 0
    queue.put((goalY, goalX), math.hypot(goalX - startX, goalY - startY), 0)

def updateVertex(x, y):
    if grid[y][x]['g'] != grid[y][x]['rhs'] and queue.includes((y, x)):
        key = calculateKey(x, y)
        queue.update((y, x), key[0], key[1])
    elif grid[y][x]['g'] != grid[y][x]['rhs'] and not queue.includes((y, x)):
        key = calculateKey(x, y)
        queue.put((y, x), key[0], key[1])
    elif grid[y][x]['g'] == grid[y][x]['rhs'] and queue.includes((y, x)):
        queue.remove((y, x))

def keyComp(A, B):
    if A[0] > B[0]:
        return 1
    elif A[0] < B[0]:
        return -1
    elif A[1] > B[1]:
        return 1
    elif A[1] < B[1]:
        return -1
    return 0

def computeShortestPath():
    while keyComp(queue.topKey(), calculateKey(startX, startY)) == -1 or grid[startY][startX]['rhs'] > grid[startY][startX]['g']:
        item = queue.peek()
        k_old = queue.topKey()
        k_new = calculateKey(item[1], item[0])
        if keyComp(k_old, k_new) == -1:
            queue.update(item, k_new[0], k_old[1])
        elif grid[item[0]][item[1]]['g'] > grid[item[0]][item[1]]['rhs']:
            grid[item[0]][item[1]]['g'] = grid[item[0]][item[1]]['rhs']
            queue.remove(item)
            for y,x in neighbors(item[0], item[1], False):
                if not (y == goalY and x == goalY):
                    newRhs = grid[item[0]][item[1]]['g'] + cost(item[1], item[0], x, y)
                    if newRhs < grid[y][x]['rhs']:
                        grid[y][x]['rhs'] = newRhs
                        grid[y][x]['next'] = (item[0], item[1])
                updateVertex(x, y)
        else:
            g_old = grid[item[0]][item[1]]['g']
            grid[item[0]][item[1]]['g'] = float('inf')
            for y,x in neighbors(item[0], item[1], True):
                if grid[y][x]['rhs'] == cost(item[1], item[0], x, y) + g_old:
                    if not (y == goalY and x == goalY):
                        minRhs = float('inf')
                        for yn, xn in neighbors(y, x, False):
                            if minRhs > cost(xn, yn, x, y) + grid[yn][xn]['g']:
                                minRhs = cost(xn, yn, x, y) + grid[yn][xn]['g']
                                grid[y][x]['next'] = (yn, xn)
                        grid[y][x]['rhs'] = minRhs
                updateVertex(x, y)

def cost(x1, y1, x2, y2):
    print(x1, y1, x2, y2)
    if grid[y1][x1]['open'] == 0 or grid[y2][x2]['open'] == 0:
        return float('inf')
    else:
        return math.hypot(y1 - y2, x1 - x2)

def calculateKey(x, y):
    return [min(grid[y][x]['g'], grid[y][x]['rhs']) + math.hypot(x - startX, y - startY) + km, min(grid[y][x]['g'], grid[y][x]['rhs'])]

def neighbors(y, x, includeSelf):
    print(x, y)
    for i in range(3):
        for j in range(3):
            toYield = (y + i - 1, x + j - 1)
            if toYield[0] >= len(grid) or toYield[0] < 0:
                continue
            if toYield[1] >= len(grid[0]) or toYield[1] < 0:
                continue
            if toYield[0] == y and toYield[1] == x and not includeSelf:
                continue
            yield toYield[0], toYield[1]

def main():
    initialize()
    computeShortestPath()
    nextX = startX
    nextY = startY
    while True:
        print(nextX, nextY)
        if (grid[nextY][nextX]['next'] == None):
            break
        nextX, nextY = grid[nextY][nextX]['next'][1], grid[nextY][nextX]['next'][0]

def updateGrid():
    #TODO: This
    print("NYI")

if __name__ == "__main__":
    main()