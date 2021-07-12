import numpy as np
import heapq
import math
import time
import pygame


class Node:
    def __init__(self, state=None, cost=float('inf'), costToCome=float('inf'), parent=None, collision=None):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.costToCome = costToCome
        self.collision = collision


class CoupledPlannerNode:
    def __init__(self, state=None, collision=None, parent=None, f_score=float('inf'), cost_to_come=float('inf')):
        self.state = state
        self.parent = parent
        self.collision = collision
        self.f_score = f_score
        self.cost_to_come = cost_to_come


class CoupledNode:
    def __init__(self, state=None, collision=None, parent=None, f_score=None, cost_to_go=None, cost_to_come=None):
        self.state = state
        self.parent = parent
        self.collision = collision
        self.f_score = f_score
        self.cost_to_go = cost_to_go
        self.cost_to_come = cost_to_come


def pointInValidWorkspace(point, res, radiusClearance, scale):
    x, y = point

    # --------------------------------------------------------------------------------
    #                     Checking whether point inside obstacles
    # --------------------------------------------------------------------------------
    X = np.float32([8, 12.5, 12.5, 8]) * scale / res
    Y = np.float32([9, 9, 9.5, 9.5]) * scale / res
    ptInRectangle = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                    X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([10, 10.5, 10.5, 10]) * scale / res
    Y = np.float32([7, 7, 11.5, 11.5]) * scale / res
    ptInRectangle1 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([4, 4.25, 4.25, 4]) * scale / res
    Y = np.float32([8, 8, 10.5, 10.5]) * scale / res
    ptInRectangle2 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([1.5, 3, 3, 1.5]) * scale / res
    Y = np.float32([9, 9, 9.25, 9.25]) * scale / res
    ptInRectangle3 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([16, 16.25, 16.25, 16]) * scale / res
    Y = np.float32([8, 8, 10.5, 10.5]) * scale / res
    ptInRectangle4 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([17, 18.5, 18.5, 17]) * scale / res
    Y = np.float32([9, 9, 9.25, 9.25]) * scale / res
    ptInRectangle5 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) * scale / res
    Y = np.float32([3, 3, 3.25, 3.25]) * scale / res
    ptInRectangle6 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([10.15, 10.40, 10.40, 10.15]) * scale / res
    Y = np.float32([0.8, 0.8, 2.3, 2.3]) * scale / res
    ptInRectangle7 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) * scale / res
    Y = np.float32([15, 15, 15.25, 15.25]) * scale / res
    ptInRectangle8 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([10.15, 10.40, 10.40, 10.15]) * scale / res
    Y = np.float32([16, 16, 17.5, 17.5]) * scale / res
    ptInRectangle9 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    if ptInRectangle or ptInRectangle1 or ptInRectangle2 or ptInRectangle3 or ptInRectangle4 or \
            ptInRectangle5 or ptInRectangle6 or ptInRectangle7 or ptInRectangle8 or ptInRectangle9:
        return False
    return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, scale, r=1, radiusClearance=0):
    col = math.floor(800 / r)
    row = math.floor(800 / r)
    newState = list(newState)
    if not isinstance(newState[0], list):
        if newState[0] < 0 or newState[0] > col or newState[1] < 0 or newState[1] > row:
            return False
        return pointInValidWorkspace(newState[0:2], r, radiusClearance, scale)
    else:
        check = True
        for i in range(len(newState)):
            check = check or newState[i][0] < 0 or newState[i][0] > col or newState[i][1] < 0 or newState[i][1] > row
            if check:
                check = pointInValidWorkspace(newState[i][0:2], r, radiusClearance, scale)
            else:
                return False
        return check


# prints solution path
def printPath(node):
    l = []
    current = node
    while current:
        l.append(current.state)
        current = current.parent
    return l


3


def normalize(startPosition, startOrientation, threshDistance=0.5, threshAngle=30):
    x, y = startPosition
    t = startOrientation
    x = round(x / threshDistance) * threshDistance
    y = round(y / threshDistance) * threshDistance
    t = round(t / threshAngle) * threshAngle
    return [x, y, t]


# Calculating the Euclidean distance
def distance(startPosition, goalPosition):
    sx, sy = startPosition
    gx, gy = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)


# generates optimal path for robot
def Astar(q, startPosition, startOrientation, goalPosition, nodesExplored, scale, threshDistance=0.5, threshAngle=30,
          radiusClearance=0):
    # normalize goal and start positions
    sx, sy, st = normalize(startPosition, startOrientation, threshDistance, threshAngle)
    gx, gy, gt = normalize(goalPosition, 0, threshDistance, threshAngle)

    # Initializing root node
    key = str(sx) + str(sy) + str(st)
    root = Node(np.array([sx, sy, st]), 0.0, 0.0, None)
    if key not in nodesExplored:
        nodesExplored[key] = root

    count = 1
    heapq.heappush(q, (root.cost, count, root))

    while len(q) > 0:
        _, _, currentNode = heapq.heappop(q)
        if distance(currentNode.state[0:2], goalPosition) <= 3 * 1.5:
            sol = printPath(currentNode)
            return [True, sol]
        angle = 360 // threshAngle
        for theta in range(angle):
            x, y, t = currentNode.state
            newOrientation = math.radians((threshAngle * theta + t) % 360)
            newPosX = threshDistance * math.cos(newOrientation) + x
            newPosY = threshDistance * math.sin(newOrientation) + y
            newState = np.array(normalize([newPosX, newPosY], newOrientation, threshDistance, threshAngle))
            s = str(newState[0]) + str(newState[1]) + str(newState[2])

            if s not in nodesExplored:
                if isSafe(newState, scale, 1, radiusClearance):
                    newCostToCome = currentNode.costToCome + distance([newState[0], newState[1]], [x, y])
                    newCost = newCostToCome + distance([newState[0], newState[1]], [gx, gy])

                    newNode = Node(state=newState, cost=newCost, costToCome=newCostToCome, parent=currentNode)
                    nodesExplored[s] = newNode
                    heapq.heappush(q, (newNode.cost, count, newNode))
                    count += 1
            else:
                if nodesExplored[s].collision is None or (
                        isinstance(nodesExplored[s].collision, list) and len(nodesExplored[s].collision) == 0):
                    if (nodesExplored[s].cost > currentNode.costToCome + distance([newState[0], newState[1]],
                                                                                  [x, y]) + distance(
                        [newState[0], newState[1]], [gx, gy])):
                        nodesExplored[s].costToCome = currentNode.costToCome + distance([newState[0], newState[1]],
                                                                                        [x, y])
                        nodesExplored[s].cost = nodesExplored[s].costToCome + distance([newState[0], newState[1]],
                                                                                       [gx, gy])
                        nodesExplored[s].parent = currentNode

    return [False, None]  # checks whether next action is near an obstacle or ill defined


def determineCollision(robotPosition):
    collisionSet = []
    for i in range(len(robotPosition) - 1):
        collision = []
        for j in range(i + 1, len(robotPosition)):
            if list(robotPosition[i]) == list(robotPosition[j]):
                collision.append(i)
                collision.append(j)
        collision = list(set(collision))
        if collision:
            collisionSet.append(collision)
    return collisionSet


def coupledPlanner(collision, startPosition, startOrientation, goalPosition, coupledNodesExplored, nodesExplored,
                   solPaths1,
                   iterateSolPaths1, scale, threshDistance=0.5, threshAngle=30, radiusClearance=0):
    nonCollisionRobots = np.array([Node()] * len(startPosition))
    goalChecker = {}
    solution = {}
    solution1 = {}
    nodeE = {}
    co = 0
    currentPos = startPosition.copy()
    count = [0] * len(startPosition)
    q = {}
    col = []
    for i in range(len(startPosition)):
        q[i] = []
        if i not in collision:
            s = str(solPaths1[i][iterateSolPaths1[i]][0]) + str(solPaths1[i][iterateSolPaths1[i]][1]) + str(
                solPaths1[i][iterateSolPaths1[i]][2])
            nonCollisionRobots[i] = coupledNodesExplored[s]
            iterateSolPaths1[i] -= 1
        else:
            goalChecker[i] = False
            nodeE[i] = {}
            root = Node(startPosition[i], 0.0, 0.0, None)
            s = str(startPosition[i][0]) + str(startPosition[i][1]) + str(startPosition[i][2])
            nodeE[i][s] = root
            count[i] += 1
            heapq.heappush(q[i], (root.cost, count[i], root))

    while not all(ele for ele in goalChecker.values()):
        co += 1
        # print(currentPos, determineCollision(currentPos.copy()), len(currentPos), co)
        if determineCollision(currentPos.copy()):
            col = determineCollision(currentPos.copy())
            for i in col[0]:
                s = str(currentPos[i][0]) + str(currentPos[i][1]) + str(currentPos[i][2])
                q[i].clear()
                nodesExplored[i][s].collision = col
                heapq.heappush(q[i], (nodesExplored[i][s].parent.cost, count[i], nodesExplored[i][s].parent))
                nodesExplored[i][s].parent = None
                nodeE[i].clear()
        # collision = list(set(collision + col[0]))
        # print(collision)
        for i in range(len(startPosition)):
            if i in collision:
                if not goalChecker[i]:
                    _, _, currentNode = heapq.heappop(q[i])
                    currentPos[i] = currentNode.state
                    if distance(currentNode.state[0:2], goalPosition[i][0:2]) <= 3 * 1.5:
                        solution[i] = printPath(currentNode)

                        goalChecker[i] = True
                        continue

                    angle = 360 // threshAngle
                    for theta in range(angle):
                        x, y, t = currentNode.state
                        newOrientation = math.radians((threshAngle * theta + t) % 360)
                        newPosX = threshDistance * math.cos(newOrientation) + x
                        newPosY = threshDistance * math.sin(newOrientation) + y
                        newState = np.array(normalize([newPosX, newPosY], newOrientation, threshDistance, threshAngle))
                        s = str(newState[0]) + str(newState[1]) + str(newState[2])

                        if s not in nodeE[i]:
                            if (s in nodesExplored[i] and not nodesExplored[i][s].collision) or (
                                    s not in nodesExplored[i]):
                                if isSafe(newState, scale, 1, radiusClearance):
                                    newCostToCome = currentNode.costToCome + distance([newState[0], newState[1]],
                                                                                      [x, y])
                                    newCost = newCostToCome + distance([newState[0], newState[1]], goalPosition[i][0:2])

                                    newNode = Node(state=newState, cost=newCost, costToCome=newCostToCome,
                                                   parent=currentNode)
                                    nodesExplored[i][s] = newNode
                                    nodeE[i][s] = newNode
                                    heapq.heappush(q[i], (newNode.cost, count[i], newNode))
                                    count[i] += 1
                        else:
                            if (s in nodesExplored[i] and not nodesExplored[i][s].collision) or (
                                    s not in nodesExplored[i]):
                                if (nodeE[i][s].cost > currentNode.costToCome + distance([newState[0], newState[1]],
                                                                                         [x, y]) + distance(
                                    [newState[0], newState[1]], goalPosition[i][0:2])):
                                    nodeE[i][s].costToCome = currentNode.costToCome + distance(
                                        [newState[0], newState[1]], [x, y])
                                    nodeE[i][s].cost = nodeE[i][s].costToCome + distance([newState[0], newState[1]],
                                                                                         goalPosition[i][0:2])
                                    nodeE[i][s].parent = currentNode
                    # print(currentNode.state)
                    # print(currentPos[i])
            else:
                if iterateSolPaths1[i] > 0:
                    s = str(solPaths1[i][iterateSolPaths1[i]][0]) + str(solPaths1[i][iterateSolPaths1[i]][1]) + str(
                        solPaths1[i][iterateSolPaths1[i]][2])
                    nonCollisionRobots[i] = nodesExplored[i][s]
                    currentPos[i] = nonCollisionRobots[i].state.copy()
                    iterateSolPaths1[i] -= 1

                else:
                    goalChecker[i] = True

    return solution, nodesExplored


def updateCollsionPath(colset, previousPos, coupledNodesExplored, nodesExplored, nodesExplored1):
    for i, pos in enumerate(previousPos):
        s = str(pos[0]) + str(pos[1]) + str(pos[2])

        while nodesExplored1[i][s].parent is not None:
            for collision in colset:
                if i in collision:
                    if coupledNodesExplored[s].collision:
                        for col in coupledNodesExplored[s].collision:
                            col = list(set(col + [i]))
                    else:
                        coupledNodesExplored[s].collision = colset

                    if nodesExplored[s].collision:
                        for col in nodesExplored[s].collision:
                            col = list(set(col + [i]))
                    else:
                        nodesExplored[s].collision = colset

                    if nodesExplored1[i][s].collision:
                        for col in nodesExplored1[i][s].collision:
                            col = list(set(col + [i]))
                    else:
                        nodesExplored1[i][s].collision = colset

            st = nodesExplored1[i][s].parent.state
            s = str(st[0]) + str(st[1]) + str(st[2])


def subdimensionalExpansion(solPaths, nodesExplored, nodesExplored1, iterateSolPaths, scale, threshDistance,
                            threshAngle,
                            radiusClearance):
    currentPos = []
    sol = []
    nodeE = []
    startPosition = []
    goalPosition = []
    previousPos = []
    colset = []
    count = -1
    exp = False
    previousNode = [Node()] * len(solPaths)
    node = [Node()] * len(solPaths)
    coupledNodesExplored = {}
    solPaths1 = solPaths.copy()
    iterateSolPaths1 = iterateSolPaths.copy()
    for index, path in enumerate(solPaths):
        startPosition.append(list(path[iterateSolPaths[index]]))
        goalPosition.append(list(path[0]))

    while not all(ele == 0 for ele in iterateSolPaths):
        previousPos = currentPos.copy()
        currentPos.clear()
        for index, path in enumerate(solPaths):
            currentPos.append(list(path[iterateSolPaths[index]]))
        colset = determineCollision(currentPos)
        count += 1

        if count == 0:
            previousPos = currentPos
        if not colset:
            for i, pos in enumerate(currentPos):
                s = str(pos[0]) + str(pos[1]) + str(pos[2])
                if count == 0:
                    node[i] = Node(state=pos, collision=colset, parent=None)
                    previousNode[i] = node[i]
                else:
                    previousNode[i] = node[i]
                    node[i] = Node(state=pos, collision=colset, parent=previousNode[i])
                if s not in nodesExplored:
                    nodesExplored[s] = node[i]
                coupledNodesExplored[s] = node[i]

                if iterateSolPaths[i] > 0:
                    iterateSolPaths[i] -= 1
        else:
            exp = True
            # print(currentPos)
            break
    for i, pos in enumerate(currentPos):
        s = str(pos[0]) + str(pos[1]) + str(pos[2])
        for collision in colset:
            if i in collision:
                node[i] = Node(state=pos, collision=colset, parent=None)
                coupledNodesExplored[s] = node[i]
                nodesExplored[s].collision = colset
                nodesExplored1[i][s].collision = colset
            else:
                while iterateSolPaths[i] > 0:
                    s = str(pos[0]) + str(pos[1]) + str(pos[2])
                    previousNode[i] = node[i]
                    node[i] = Node(state=pos, collision=[], parent=previousNode[i])
                    coupledNodesExplored[s] = node[i]
                    iterateSolPaths[i] -= 1
                    pos = solPaths[i][iterateSolPaths[i]]
                break

    if exp:
        print('Collision found')
        # print(colset)
        updateCollsionPath(colset, previousPos, coupledNodesExplored, nodesExplored, nodesExplored1)
        for collision in colset:
            a = time.time()
            sol, nodeE = coupledPlanner(collision, startPosition, 0, goalPosition, coupledNodesExplored, nodesExplored1,
                                        solPaths1,
                                        iterateSolPaths1, scale, threshDistance,
                                        30, radiusClearance)
            b = time.time()
            print(b - a)
        return exp, sol, colset[0], nodeE, currentPos
    return exp, sol, colset, nodeE, currentPos


def triangleCoordinates(start, end, triangleSize=5):
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi / 2
    rad = math.pi / 180
    coordinateList = np.array([[end[0], end[1]],
                               [end[0] + triangleSize * math.sin(rotation - 165 * rad),
                                end[1] + triangleSize * math.cos(rotation - 165 * rad)],
                               [end[0] + triangleSize * math.sin(rotation + 165 * rad),
                                end[1] + triangleSize * math.cos(rotation + 165 * rad)]])

    return coordinateList


def visualizeMStar():
    ###################################################
    #                  Parameters
    ###################################################
    clearance = 10
    radius = 0
    stepSize = 11

    threshDistance = stepSize  # Step size of movement
    res = 1  # resolution of grid
    scale = 40  # scale of grid

    # 1 Robot
    # start = [[1 * scale, 16 * scale]]  # Starting position of the robots
    # goal = [[16 * scale, 1 * scale]]  # Goal position of the robots

    # 2 Robots
    # start = [[1 * scale, 6 * scale], [6 * scale, 1 * scale]]  # Starting position of the robots
    # goal = [[14 * scale, 10 * scale], [9 * scale, 14 * scale]]  # Goal position of the robots

    # 3 Robots
    start = [[1 * scale, 16 * scale], [1 * scale, 6 * scale],
             [6 * scale, 1 * scale]]  # Starting position of the robots
    goal = [[2 * scale, 8 * scale], [14 * scale, 10 * scale],
            [9 * scale, 14 * scale]]  # Goal position of the robots

    # 4 Robots
    # start = [[1 * scale, 16 * scale], [1 * scale, 6 * scale],
    #          [6 * scale, 1 * scale], [19 * scale, 1 * scale]]  # Starting position of the robots
    # goal = [[2 * scale, 8 * scale], [14 * scale, 10 * scale],
    #         [9 * scale, 14 * scale], [5 * scale, 16 * scale]]  # Goal position of the robots

    # 5 Robots
    # start = [[1 * scale, 16 * scale], [1 * scale, 6 * scale],
    #          [6 * scale, 1 * scale], [19 * scale, 1 * scale], [17 * scale, 4 * scale]]  # Starting position of the robots
    # goal = [[2 * scale, 8 * scale], [14 * scale, 10 * scale],
    #         [9 * scale, 14 * scale], [5 * scale, 16 * scale], [19 * scale, 19 * scale]]  # Goal position of the robots

    # 6 Robots
    # start = [[1 * scale, 16 * scale], [1 * scale, 6 * scale],
    #          [6 * scale, 1 * scale], [19 * scale, 1 * scale], [17 * scale, 4 * scale], [1 * scale, 19 * scale]]  # Starting position of the robots
    # goal = [[2 * scale, 8 * scale], [14 * scale, 10 * scale],
    #         [9 * scale, 14 * scale], [5 * scale, 16 * scale], [19 * scale, 19 * scale], [14 * scale, 16 * scale]]  # Goal position of the robots

    # 7 Robots
    # start = [[1 * scale, 16 * scale], [1 * scale, 6 * scale],
    #          [6 * scale, 1 * scale], [19 * scale, 1 * scale],
    #          [17 * scale, 4 * scale], [1 * scale, 19 * scale], [2 * scale, 2 * scale]]  # Starting position of the robots
    # goal = [[2 * scale, 8 * scale], [14 * scale, 10 * scale],
    #         [9 * scale, 14 * scale], [5 * scale, 16 * scale],
    #         [19 * scale, 19 * scale], [14 * scale, 16 * scale], [14 * scale, 3 * scale]]  # Goal position of the robots

    drawing = True
    threshAngle = 90  # Angle between actions
    startOrientation = 0
    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    lred = (255, 102, 102)
    green = (0, 102, 0)
    lgreen = (153, 255, 153)
    orange = (255, 165, 0)
    dorange = (240, 94, 35)
    blue = (0, 0, 255)
    lblue = (153, 204, 255)
    purple = (75, 0, 130)
    yellow = (255, 255, 0)
    pink = (255,192,203)
    dpink = (199,21,133)
    gray = (220,220,220)
    dgray = (105,105,105)
    cyan = (0, 255, 255)
    maroon = (255,160,122)
    dmaroon = (128, 0, 0)
    pathColours = [blue, red, green, dmaroon, orange, dpink, dgray]
    colors = [lblue, lred, lgreen, maroon, dorange, pink, gray]
    solutionPaths = []
    size_x = 20
    size_y = 20
    TotalNodesExplored = {}
    TotalNodesExplored1 = {}
    totalTime = 0
    if drawing:
        pygame.init()
        gameDisplay = pygame.display.set_mode((size_x * scale, size_y * scale))
        gameDisplay.fill(white)
        pygame.display.set_caption("M* Algorithm Implementation")
        basicfont = pygame.font.SysFont('timesnewroman', 20, bold=True)
        ############################################################
        #                 Display Obstacles
        ############################################################
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 8), int(scale * 9), int(scale * 4.5), int(scale * 0.5)])  # plus
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 10), int(scale * 7), int(scale * 0.5), int(scale * 4.5)])  # plus
        pygame.draw.rect(gameDisplay, black, [int(scale * 4), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 1.5), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 16), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 17), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
        pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 3), int(scale * 2.5), int(scale * 0.25)])  # -
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 10.15), int(scale * 0.8), int(scale * 0.25), int(scale * 1.5)])  # |
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 9), int(scale * 15), int(scale * 2.5), int(scale * 0.25)])  # -
        pygame.draw.rect(gameDisplay, black,
                         [int(scale * 10.15), int(scale * 16), int(scale * 0.25), int(scale * 1.5)])  # |

        ############################################################
        #          Display start and end points of the robots
        ############################################################
        for i in range(len(start)):
            pygame.draw.circle(gameDisplay, pathColours[i], start[i], 0.1 * scale)
            pygame.draw.circle(gameDisplay, pathColours[i], goal[i], 0.1 * scale)
            text = basicfont.render('s' + str(i + 1), False, pathColours[i])
            text1 = basicfont.render('g' + str(i + 1), False, pathColours[i])
            gameDisplay.blit(text, (start[i][0] + 5, start[i][1] + 5))
            gameDisplay.blit(text1, (goal[i][0] + 5, goal[i][1] + 5))

        pygame.display.update()
        pygame.time.delay(500)
    ############################################################
    #          Draw Explored Nodes and solution path
    ############################################################
    for i in range(len(start)):
        nodesExplored = {}
        q = []
        startPosition = np.round((np.array(start[i])) / res)
        goalPosition = np.round((np.array(goal[i])) / res)
        if not isSafe(startPosition, scale, res, clearance + radius) or not isSafe(goalPosition, scale, res,
                                                                                   clearance + radius):
            print('Start or goal configuration of robot ' + str(i + 1) + ' is not in a valid workspace')

        else:
            print('Exploring workspace for robot ' + str(i + 1))
            startTime = time.time()  # Start time of simulation
            success, solution = Astar(q, startPosition, startOrientation, goalPosition, nodesExplored, scale,
                                      threshDistance,
                                      threshAngle, clearance + radius)
            endTime = time.time()
            TotalNodesExplored.update(nodesExplored)
            TotalNodesExplored1[i] = nodesExplored
            #############################################
            #      Drawing
            #############################################
            if success:
                solutionPaths.append(solution)
                print('Optimal path found for robot ' + str(i + 1))

                print("Total time taken for exploring nodes " + str(endTime - startTime) + " seconds.")
                totalTime += endTime - startTime
                print('-------------------------')
                if drawing:
                    draw = True
                    while draw:
                        for event in pygame.event.get():
                            if event.type == pygame.QUIT:
                                pygame.quit()
                                quit()

                        # draw nodesExplored
                        for s in nodesExplored:
                            if nodesExplored[s].parent:
                                pt = nodesExplored[s].state[0:2]
                                ptParent = nodesExplored[s].parent.state[0:2]
                                x, y = pt * res
                                x2, y2 = ptParent * res
                                # draw explored nodes
                                pygame.draw.line(gameDisplay, colors[i], (x2, y2), (x, y), 1)
                                triangle = triangleCoordinates([x2, y2], [x, y], 5)
                                pygame.draw.polygon(gameDisplay, colors[i],
                                                    [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])
                            # draw start and goal locations
                            pygame.draw.rect(gameDisplay, colors[i],
                                             (int(startPosition[0] * res * scale), int(startPosition[1] * res * scale),
                                              int(res * scale), int(res * scale)))

                            pygame.draw.circle(gameDisplay, colors[i],
                                               (int(goalPosition[0] * res * scale), int(goalPosition[1] * res * scale)),
                                               math.floor(3 * 1.5 * res * scale))

                            pygame.draw.rect(gameDisplay, white,
                                             (int(goalPosition[0] * res * scale), int(goalPosition[1] * res * scale),
                                              int(res * scale), int(res * scale)))

                            pygame.display.update()

                        draw = False

            else:
                solutionPaths.append(success)
    print("Total time " + str(totalTime))
    print("solution Paths " + str(len(solutionPaths)))
    print('Robots following their own individual optimal Paths')
    print()
    print()
    iterateSolutionPaths = []
    for i in range(len(solutionPaths)):
        if solutionPaths[i]:
            iterateSolutionPaths.append(len(solutionPaths[i]) - 1)
        else:
            iterateSolutionPaths.append(-1)

    iterateSolutionPathsCopy = iterateSolutionPaths.copy()
    iterateSolutionPathsCopy1 = iterateSolutionPaths.copy()
    solutionPathsCopy = solutionPaths.copy()

    failure, sol, collision, nodeE, currentPos = subdimensionalExpansion(solutionPathsCopy, TotalNodesExplored,
                                                                         TotalNodesExplored1,
                                                                         iterateSolutionPathsCopy,
                                                                         scale,
                                                                         threshDistance, 45,
                                                                         radius + clearance)

    if drawing:
        temp = [True] * len(iterateSolutionPaths)
        while not all(ele == -2 for ele in iterateSolutionPaths) and not all(not p for p in temp):
            for i in range(len(solutionPaths)):
                if list(solutionPaths[i][iterateSolutionPaths[i]]) == currentPos[i] and failure:
                    pt = solutionPaths[i][iterateSolutionPaths[i]][0:2]
                    x, y = pt[0] * res, pt[1] * res
                    pygame.draw.circle(gameDisplay, pathColours[i], (int(x * res), int(y * res)),
                                       math.floor(3 * 1.5 * res))
                    pygame.time.delay(50)
                    pygame.display.update()
                    iterateSolutionPaths[i] -= 1
                    temp[i] = False
                else:
                    if iterateSolutionPaths[i] != -2:
                        if iterateSolutionPaths[i] == -1:
                            print("There is no Path for Robot " + str(i + 1))
                            iterateSolutionPaths[i] = -2
                        elif iterateSolutionPaths[i] >= 0:
                            pt = solutionPaths[i][iterateSolutionPaths[i]][0:2]
                            x, y = pt[0] * res, pt[1] * res
                            pygame.draw.circle(gameDisplay, pathColours[i], (int(x * res), int(y * res)),
                                               math.floor(3 * 1.5 * res))
                            pygame.time.delay(50)
                            iterateSolutionPaths[i] -= 1
                            if iterateSolutionPaths[i] == 0:
                                print("Robot " + str(i + 1) + " reached its goal")
                                iterateSolutionPaths[i] = -2
                            pygame.display.update()

    pygame.time.delay(1000)
    if failure:
        s = ''
        for i in collision:
            s += str(i + 1) + ' '

        print("--------------------")
        print('Robot - Robot collision detected between robots ' + s)
        print('Starting subdimesional Expansion')
        print('Running Back propogation and updating Collision list')
        temp = []
        for i in range(len(iterateSolutionPaths)):
            if i in collision:
                temp.append(False)
            else:
                temp.append(True)

        if drawing:
            while not all(ele for ele in temp):
                for i in range(len(iterateSolutionPaths)):
                    if i in collision:
                        if iterateSolutionPaths[i] != iterateSolutionPathsCopy1[i]:
                            pt = solutionPaths[i][iterateSolutionPaths[i]][0:2]
                            x, y = pt[0] * res, pt[1] * res
                            iterateSolutionPaths[i] += 1
                            pygame.draw.circle(gameDisplay, yellow, (int(x * res), int(y * res)),
                                               math.floor(3 * 1.5 * res))
                            pygame.time.delay(50)
                            pygame.display.update()
                        else:
                            temp[i] = True

            pygame.time.delay(500)
        # pygame.quit()
        print()
        print('Implementing coupled planner for robots ' + s)
        print()
        print('Robots following collision free path')
        if drawing:
            gameDisplay.fill(white)
            pygame.display.set_caption("M* Algorithm Implementation")
            basicfont = pygame.font.SysFont('timesnewroman', 20, bold=True)
            ############################################################
            #                 Display Obstacles
            ############################################################
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 8), int(scale * 9), int(scale * 4.5), int(scale * 0.5)])  # plus
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 10), int(scale * 7), int(scale * 0.5), int(scale * 4.5)])  # plus
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 4), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 1.5), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 16), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 17), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 9), int(scale * 3), int(scale * 2.5), int(scale * 0.25)])  # -
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 10.15), int(scale * 0.8), int(scale * 0.25), int(scale * 1.5)])  # |
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 9), int(scale * 15), int(scale * 2.5), int(scale * 0.25)])  # -
            pygame.draw.rect(gameDisplay, black,
                             [int(scale * 10.15), int(scale * 16), int(scale * 0.25), int(scale * 1.5)])  # |

            pygame.display.update()

        solutionPaths2 = solutionPathsCopy.copy()
        sol = list(np.load('sol.npy'))
        sol.reverse()
        for i in range(len(solutionPaths2)):
            if i in collision:
                solutionPaths2[i] = sol.pop(0)


        iterateSolutionPaths2 = []
        if drawing:
            for i in range(len(start)):
                pygame.draw.circle(gameDisplay, black, start[i], 0.1 * scale)
                pygame.draw.circle(gameDisplay, black, goal[i], 0.1 * scale)
                text = basicfont.render('s' + str(i + 1), False, black)
                text1 = basicfont.render('g' + str(i + 1), False, black)
                gameDisplay.blit(text, (start[i][0] + 5, start[i][1] + 5))
                gameDisplay.blit(text1, (goal[i][0] + 5, goal[i][1] + 5))

            pygame.display.update()
            pygame.time.delay(500)
            for i in range(len(start)):
                # if i not in collision:
                startPosition = np.round((np.array(start[i])) / res)
                goalPosition = np.round((np.array(goal[i])) / res)

                draw = True
                while draw:
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            pygame.quit()
                            quit()

                    # draw nodesExplored
                    for s in nodeE[i]:
                        if nodeE[i][s].parent:
                            pt = nodeE[i][s].state[0:2]
                            ptParent = nodeE[i][s].parent.state[0:2]
                            x, y = pt * res
                            x2, y2 = ptParent * res
                            # draw explored nodes
                            if i in collision:
                                pygame.draw.line(gameDisplay, yellow, (x2, y2), (x, y), 1)
                                triangle = triangleCoordinates([x2, y2], [x, y], 5)
                                pygame.draw.polygon(gameDisplay, yellow,
                                                    [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])
                            else:
                                pygame.draw.line(gameDisplay, colors[i], (x2, y2), (x, y), 1)
                                triangle = triangleCoordinates([x2, y2], [x, y], 5)
                                pygame.draw.polygon(gameDisplay, colors[i],
                                                    [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])
                        # draw start and goal locations
                        pygame.draw.rect(gameDisplay, colors[i],
                                         (int(startPosition[0] * res * scale), int(startPosition[1] * res * scale),
                                          int(res * scale), int(res * scale)))

                        pygame.draw.circle(gameDisplay, colors[i],
                                           (int(goalPosition[0] * res * scale), int(goalPosition[1] * res * scale)),
                                           math.floor(3 * 1.5 * res * scale))

                        pygame.draw.rect(gameDisplay, white,
                                         (int(goalPosition[0] * res * scale), int(goalPosition[1] * res * scale),
                                          int(res * scale), int(res * scale)))

                        pygame.display.update()

                    draw = False

            for i in range(len(solutionPaths2)):
                iterateSolutionPaths2.append(len(solutionPaths2[i]) - 1)

            print(iterateSolutionPaths2)

            # draw solution path

            while not all(ele == -2 for ele in iterateSolutionPaths2):
                for i in range(len(solutionPaths2)):
                    if iterateSolutionPaths2[i] != -2:
                        if iterateSolutionPaths2[i] == -1:
                            print("There is no Path for Robot " + str(i + 1))
                            iterateSolutionPaths2[i] = -2
                        elif iterateSolutionPaths2[i] >= 0:
                            pt = solutionPaths2[i][iterateSolutionPaths2[i]][0:2]
                            x, y = pt[0] * res, pt[1] * res
                            pygame.draw.circle(gameDisplay, pathColours[i], (int(x * res), int(y * res)),
                                               math.floor(3 * 1.5 * res))
                            pygame.time.delay(50)
                            iterateSolutionPaths2[i] -= 1
                            if iterateSolutionPaths2[i] == 0:
                                print("Robot " + str(i + 1) + " reached its goal")
                                iterateSolutionPaths2[i] = -2
                            pygame.display.update()

            pygame.time.delay(4000)
            pygame.quit()


def main():
    visualizeMStar()


if __name__ == "__main__":
    main()
