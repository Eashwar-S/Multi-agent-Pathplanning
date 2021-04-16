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
    Y = np.float32([7, 7, 14.5, 14.5]) * scale / res
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

    X = np.float32([17, 17.5, 17.5, 17]) * scale / res
    Y = np.float32([9, 9, 9.25, 9.25]) * scale / res
    ptInRectangle5 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) * scale / res
    Y = np.float32([3, 3, 3.25, 3.25]) * scale / res
    ptInRectangle6 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     X[1] + radiusClearance / res >= x >= X[3] - radiusClearance / res

    X = np.float32([10.15, 10.35, 10.35, 10.15]) * scale / res
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


def printCoupledPath(node, index):
    l = []
    current = node
    count = 0
    previousPoint = 0
    while current:
        if count == 0:
            l.append(current.state[index])
            previousPoint = current.state[index]
            current = current.parent
            count += 1
        else:
            if previousPoint == current.state[index]:
                current = current.parent
            else:
                l.append(current.state[index])
                previousPoint = current.state[index]
                current = current.parent
    print(l)
    return l


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
    key = str(sx) + str(sy)  # + str(st)
    root = Node(np.array([sx, sy, st]), 0.0, 0.0, None)
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
            s = str(newState[0]) + str(newState[1])  # + str(newState[2])

            if s not in nodesExplored:
                if isSafe(newState, scale, 1, radiusClearance):
                    newCostToCome = currentNode.costToCome + distance([newState[0], newState[1]], [x, y])
                    newCost = newCostToCome + distance([newState[0], newState[1]], [gx, gy])

                    newNode = Node(state=newState, cost=newCost, costToCome=newCostToCome, parent=currentNode)
                    nodesExplored[s] = newNode
                    heapq.heappush(q, (newNode.cost, count, newNode))
                    count += 1
            else:
                if nodesExplored[s].collision is None or (isinstance(nodesExplored[s].collision, list) and len(nodesExplored[s].collision) == 0):
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


def checkCollision(state, nonCollisionRobots, collision):
    state1 = state.copy()
    robotPosition = [[]] * (nonCollisionRobots.shape[0])
    for i in range(nonCollisionRobots.shape[0]):
        if i not in collision:
            robotPosition[i] = nonCollisionRobots[i].state
        else:

            robotPosition[i] = state1.pop(0)

    return determineCollision(robotPosition)


def differentPossibleNeighbours(possibleNeighbours):
    totalNeighbours = []

    l1 = possibleNeighbours[0]
    l2 = possibleNeighbours[1]
    for i, ele in enumerate(l1):
        for ele1 in l2:
            totalNeighbours.append([ele, ele1])
    return totalNeighbours


def updateCollsionList(colset, state, coupledNodesExplored):
    for st in state:
        s = str(st[0]) + str(st[1]) + str(st[2])
        if s in coupledNodesExplored:
            if coupledNodesExplored[s].collision is None:
                coupledNodesExplored[s].collision = colset
            else:
                for collision in colset:
                    coupledNodesExplored[s].collision.append(collision)

    pass


def coupledPlanner(collision, startPosition, startOrientation, goalPosition, coupledNodesExplored, solPaths1,
                   iterateSolPaths1, scale, threshDistance=0.5, threshAngle=30, radiusClearance=0):
    nonCollisionRobots = np.array([CoupledPlannerNode()] * len(startPosition))
    startConfig = []
    goalConfig = []
    goalChecker = [False] * len(collision)
    solution = [[]] * len(collision)
    visited = []
    q = []
    for index, sol in enumerate(solPaths1):
        if index not in collision:
            s = str(sol[iterateSolPaths1[index]][0]) + str(sol[iterateSolPaths1[index]][1]) + str(
                sol[iterateSolPaths1[index]][2])
            nonCollisionRobots[index] = coupledNodesExplored[s]
            iterateSolPaths1[index] -= 1
        else:
            startConfig.append(list(sol[iterateSolPaths1[index]]))
            goalConfig.append(list(sol[0]))

    # Initializing root nodes
    node = []
    for i in range(len(startConfig)):
        node1 = CoupledPlannerNode(state=startConfig[i], collision=None, parent=None)
        node.append(node1)

    root = CoupledNode(node, collision=None, cost_to_come=[0.0, 0.0], cost_to_go=[0.0, 0.0], parent=None,
                       f_score=[0.0, 0.0])
    count = 1
    heapq.heappush(q, (sum(root.cost_to_go), sum(root.f_score), root))
    # while len(q) > 0:
    for d in range(40):
        _, _, currentNode = heapq.heappop(q)
        # print(currentNode.cost_to_come)
        print(currentNode.cost_to_go)
        print(distance(currentNode.state[0].state[0:2], goalConfig[0][0:2]),
              distance(currentNode.state[1].state[0:2], goalConfig[1][0:2]))
        #     print(currentNode.f_score)
        # for i in range(nonCollisionRobots.shape[0]):
        #     if nonCollisionRobots[i].state is not None:
        #         currentNonCollisionRobotState.append(nonCollisionRobots[i].state)
        # TODO Uncomment
        for i in range(len(startConfig)):
            if distance(currentNode.state[i].state[0:2], goalConfig[i][0:2]) <= 500:
                # TODO Create a printPath function
                solution[i] = printCoupledPath(currentNode, i)
                goalChecker[i] = True
        if all(ele for ele in goalChecker):
            return [True, solution]
        #     c = 0
        possibleNeightbours = [[]] * len(collision)
        for i in range(len(startConfig)):
            x, y, t = currentNode.state[i].state
            neighbour = []
            for theta in range(12):
                if not goalChecker[i]:
                    newOrientation = math.radians((threshAngle * theta + t) % 360)
                    newPosX = threshDistance * math.cos(newOrientation) + x
                    newPosY = threshDistance * math.sin(newOrientation) + y
                    newState = normalize([newPosX, newPosY], newOrientation, threshDistance, threshAngle)
                else:
                    newState = [x, y, t]
                x, y, t = newState
                s = str(newState[0]) + str(newState[1]) + str(newState[2])
                if s not in coupledNodesExplored:
                    if isSafe(newState, scale, 1, radiusClearance) and newState not in possibleNeightbours[i]:
                        coupledNodesExplored[s] = CoupledPlannerNode(state=newState, parent=coupledNodesExplored[
                            str(currentNode.state[i].state[0]) + str(currentNode.state[i].state[1]) + str(
                                currentNode.state[i].state[2])])
                        neighbour.append(newState)
                else:
                    if coupledNodesExplored[s].collision:
                        neighbour.append(newState)
            possibleNeightbours[i] = neighbour

        totalNeighbours = differentPossibleNeighbours(possibleNeightbours)
        for state in totalNeighbours:
            # if state not in visited.values():
            colset = checkCollision(state, nonCollisionRobots, collision)
            updateCollsionList(colset, state, coupledNodesExplored)
            if not colset:
                new_f_score = [0.0] * len(state)
                new_cost_to_go = [0.0] * len(state)
                new_cost_to_come = [0.0] * len(state)
                tempnode = []
                if state not in visited:
                    for i in range(len(state)):
                        new_cost_to_come[i] = distance(currentNode.state[i].state[0:2], state[i][0:2]) + \
                                              currentNode.cost_to_come[i]
                        new_cost_to_go[i] = distance(state[i][0:2], goalConfig[i][0:2])
                        new_f_score[i] = new_cost_to_come[i] + new_cost_to_go[i]
                        tempnode1 = CoupledPlannerNode(state[i], collision=None, parent=currentNode.state[i],
                                                       cost_to_come=new_cost_to_come[i], f_score=new_f_score[i])
                        tempnode.append(tempnode1)
                        s = str(state[i][0]) + str(state[i][1]) + str(state[i][2])
                        coupledNodesExplored[s] = tempnode1
                    visited.append(state)
                else:
                    for i in range(len(state)):
                        s = str(state[i][0]) + str(state[i][1]) + str(state[i][2])
                        temp = distance(currentNode.state[i].state[0:2], state[i][0:2])
                        temp1 = distance(state[i][0:2], goalConfig[i][0:2])
                        if coupledNodesExplored[s].f_score > currentNode.cost_to_come[i] + temp + temp1:
                            coupledNodesExplored[s].cost_to_come = currentNode.cost_to_come[i] + temp
                            coupledNodesExplored[s].f_score = coupledNodesExplored[s].cost_to_come + temp1
                            coupledNodesExplored[s].parent = currentNode.state[i]
                            tempnode.append(coupledNodesExplored[s])
                newNode = CoupledNode(state=tempnode, collision=None, parent=currentNode, f_score=new_f_score,
                                      cost_to_go=new_cost_to_go, cost_to_come=new_cost_to_come)
                # print(newNode.state, newNode.f_score, d, goalConfig)
                heapq.heappush(q, (sum(newNode.cost_to_go), sum(newNode.f_score), newNode))
                count += 1
        # else:
        #     visited[False] = state
        # else:
        #     print('Eas')
        #     node = None
        #     for key, val in visited.items():
        #         if val == state:
        #             node = key
        #             break
        #     if node:
        #         temp = 0
        #         temp1 = 0
        #         for i in range(len(startConfig)):
        #             temp += distance(currentNode.state[i][0:2], state[i][0:2])
        #             temp1 += distance(state[i][0:2], goalConfig[i][0:2])
        #
        #         if node.f_score > currentNode.cost_to_come + temp + temp1:
        #             node.cost_to_come = currentNode.cost_to_come + temp
        #             node.f_score = node.cost_to_come + temp1
        #             node.parent = currentNode

        for index, sol in enumerate(solPaths1):
            if index not in collision:
                if iterateSolPaths1[index] > 0:
                    s = str(sol[iterateSolPaths1[index]][0]) + str(sol[iterateSolPaths1[index]][1]) + str(
                        sol[iterateSolPaths1[index]][2])
                    nonCollisionRobots[index] = coupledNodesExplored[s]
                    iterateSolPaths1[index] -= 1

    # for sol in solution:
    #     print(len(sol))
    #
    # print()
    # for i, sol in enumerate(solPaths1):
    #     print(len(sol))
    return [False, None]


def updateCollsionPath(colset, previousPos, coupledNodesExplored, nodesExplored):
    for i, pos in enumerate(previousPos):
        s = str(pos[0]) + str(pos[1]) + str(pos[2])
        while coupledNodesExplored[s].parent is not None:
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

            st = coupledNodesExplored[s].parent.state
            s = str(st[0]) + str(st[1]) + str(st[2])


def subdimensionalExpansion(solPaths, nodesExplored, iterateSolPaths, scale, threshDistance, threshAngle,
                            radiusClearance):
    currentPos = []
    startPosition = []
    goalPosition = []
    previousPos = []
    colset = []
    count = -1
    # previousNode = [CoupledPlannerNode()] * len(solPaths)
    # node = [CoupledPlannerNode()] * len(solPaths)
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
            # if iterateSolPaths[index] > 0:
            #     iterateSolPaths[index] -= 1
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
            break
    currentPos1 = currentPos
    for i, pos in enumerate(currentPos):
        s = str(pos[0]) + str(pos[1]) + str(pos[2])
        for collision in colset:
            if i in collision:
                node[i] = Node(state=pos, collision=colset, parent=None)
                coupledNodesExplored[s] = node[i]
                nodesExplored[s].collision = colset
            else:
                while iterateSolPaths[i] > 0:
                    s = str(pos[0]) + str(pos[1]) + str(pos[2])
                    previousNode[i] = node[i]
                    node[i] = Node(state=pos, collision=[], parent=previousNode[i])
                    coupledNodesExplored[s] = node[i]
                    iterateSolPaths[i] -= 1
                    pos = solPaths[i][iterateSolPaths[i]]
                break

    updateCollsionPath(colset, previousPos, coupledNodesExplored, nodesExplored)
    for collision in colset:
        [success, sol] = coupledPlanner(collision, startPosition, 0, goalPosition, coupledNodesExplored, solPaths1,
                                        iterateSolPaths1, scale, threshDistance,
                                        30, radiusClearance)
        print(success)
        print(sol)

    return True


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
    startTime = time.time()  # Start time of simulation
    threshDistance = stepSize  # Step size of movement
    res = 1  # resolution of grid
    scale = 40  # scale of grid
    start = [[1 * scale, 16 * scale], [1 * scale, 6 * scale],
             [6 * scale, 1 * scale]]  # Starting position of the robots
    goal = [[2 * scale, 8 * scale], [14 * scale, 10 * scale],
            [9 * scale, 14 * scale]]  # Goal position of the robots

    # start = [[1 * scale, 16 * scale], [1 * scale, 1 * scale], [1 * scale, 6 * scale],
    #          [6 * scale, 1 * scale]]  # Starting position of the robots
    # goal = [[2 * scale, 8 * scale], [16 * scale, 16 * scale], [14 * scale, 10 * scale],
    #         [9 * scale, 14 * scale]]  # Goal position of the robots

    threshAngle = 90  # Angle between actions
    startOrientation = 0
    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)
    orange = (255, 140, 0)
    blue = (0, 0, 255)
    purple = (75, 0, 130)
    yellow = (255, 255, 0)
    pink = (255, 20, 147)
    cyan = (0, 255, 255)
    maroon = (128, 0, 0)
    pathColours = [red, purple, pink, maroon]
    colors = [green, orange, yellow, cyan]
    solutionPaths = []
    size_x = 20
    size_y = 20
    q = []
    TotalNodesExplored = {}
    # pygame.init()
    # gameDisplay = pygame.display.set_mode((size_x * scale, size_y * scale))
    # gameDisplay.fill(white)
    # pygame.display.set_caption("M* Algorithm Implementation")
    # basicfont = pygame.font.SysFont('timesnewroman', 20, bold=True)
    ############################################################
    #                 Display Obstacles
    ############################################################
    # pygame.draw.rect(gameDisplay, black, [int(scale * 8), int(scale * 9), int(scale * 4.5), int(scale * 0.5)])  # plus
    # pygame.draw.rect(gameDisplay, black, [int(scale * 10), int(scale * 7), int(scale * 0.5), int(scale * 4.5)])  # plus
    # pygame.draw.rect(gameDisplay, black, [int(scale * 4), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
    # pygame.draw.rect(gameDisplay, black, [int(scale * 1.5), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
    # pygame.draw.rect(gameDisplay, black, [int(scale * 16), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
    # pygame.draw.rect(gameDisplay, black, [int(scale * 17), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
    # pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 3), int(scale * 2.5), int(scale * 0.25)])  # -
    # pygame.draw.rect(gameDisplay, black,
    #                  [int(scale * 10.15), int(scale * 0.8), int(scale * 0.25), int(scale * 1.5)])  # |
    # pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 15), int(scale * 2.5), int(scale * 0.25)])  # -
    # pygame.draw.rect(gameDisplay, black,
    #                  [int(scale * 10.15), int(scale * 16), int(scale * 0.25), int(scale * 1.5)])  # |

    ############################################################
    #          Display start and end points of the robots
    ############################################################
    # for i in range(len(start)):
    #     pygame.draw.circle(gameDisplay, black, start[i], 0.1 * scale)
    #     pygame.draw.circle(gameDisplay, black, goal[i], 0.1 * scale)
    #     text = basicfont.render('s' + str(i + 1), False, black)
    #     text1 = basicfont.render('g' + str(i + 1), False, black)
    #     gameDisplay.blit(text, (start[i][0] + 5, start[i][1] + 5))
    #     gameDisplay.blit(text1, (goal[i][0] + 5, goal[i][1] + 5))
    #
    # pygame.display.update()
    # pygame.time.delay(500)
    ############################################################
    #          Draw Explored Nodes and solution path
    ############################################################
    for i in range(len(start)):
        # pygame.draw.circle(gameDisplay, black, start[i], 0.1 * scale)
        # pygame.draw.circle(gameDisplay, black, goal[i], 0.1 * scale)
        # text = basicfont.render('s' + str(i + 1), False, black)
        # text1 = basicfont.render('g' + str(i + 1), False, black)
        # gameDisplay.blit(text, (start[i][0]+5, start[i][1]+5))
        # gameDisplay.blit(text1, (goal[i][0]+5, goal[i][1]+5))
        nodesExplored = {}
        q = []
        startPosition = np.round((np.array(start[i])) / res)
        goalPosition = np.round((np.array(goal[i])) / res)
        if not isSafe(startPosition, scale, res, clearance + radius) or not isSafe(goalPosition, scale, res,
                                                                                   clearance + radius):
            print('Start or goal configuration of robot ' + str(i + 1) + ' is not in a valid workspace')

        else:
            print('Exploring workspace for robot ' + str(i + 1))
            success, solution = Astar(q, startPosition, startOrientation, goalPosition, nodesExplored, scale,
                                      threshDistance,
                                      threshAngle, clearance + radius)
            print(len(nodesExplored))
            TotalNodesExplored.update(nodesExplored)
            #############################################
            #      Drawing
            #############################################
            if success:
                solutionPaths.append(solution)
                print('Optimal path found for robot ' + str(i + 1))
                endTime = time.time()
                print("Total time taken for exploring nodes " + str(endTime - startTime) + " seconds.")
                draw = True
                # while draw:
                #     for event in pygame.event.get():
                #         if event.type == pygame.QUIT:
                #             pygame.quit()
                #             quit()
                #
                #     # draw nodesExplored
                #     for s in nodesExplored:
                #         if nodesExplored[s].parent:
                #             pt = nodesExplored[s].state[0:2]
                #             ptParent = nodesExplored[s].parent.state[0:2]
                #             x, y = pt * res
                #             x2, y2 = ptParent * res
                #             # draw explored nodes
                #             pygame.draw.line(gameDisplay, colors[i], (x2, y2), (x, y), 1)
                #             triangle = triangleCoordinates([x2, y2], [x, y], 5)
                #             pygame.draw.polygon(gameDisplay, colors[i],
                #                                 [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])
                #         # draw start and goal locations
                #         pygame.draw.rect(gameDisplay, colors[i],
                #                          (int(startPosition[0] * res * scale), int(startPosition[1] * res * scale),
                #                           int(res * scale), int(res * scale)))
                #
                #         pygame.draw.circle(gameDisplay, colors[i],
                #                            (int(goalPosition[0] * res * scale), int(goalPosition[1] * res * scale)),
                #                            math.floor(3 * 1.5 * res * scale))
                #
                #         pygame.draw.rect(gameDisplay, white,
                #                          (int(goalPosition[0] * res * scale), int(goalPosition[1] * res * scale),
                #                           int(res * scale), int(res * scale)))
                #
                #         pygame.display.update()
                #
                #     draw = False

            else:
                solutionPaths.append(success)

    iterateSolutionPaths = []
    for i in range(len(solutionPaths)):
        if solutionPaths[i]:
            iterateSolutionPaths.append(len(solutionPaths[i]) - 1)
        else:
            iterateSolutionPaths.append(-1)

    # nodesExplored1 = nodesExplored.copy()
    print(len(TotalNodesExplored))
    print(subdimensionalExpansion(solutionPaths, TotalNodesExplored, iterateSolutionPaths, scale, threshDistance, threshAngle,
                                radius + clearance))

    # draw solution path
    # while not all(ele == -2 for ele in iterateSolutionPaths):
    #     for i in range(len(solutionPaths)):
    #         if iterateSolutionPaths[i] != -2:
    #             if iterateSolutionPaths[i] == -1:
    #                 print("There is no Path for Robot " + str(i + 1))
    #                 iterateSolutionPaths[i] = -2
    #             elif iterateSolutionPaths[i] >= 0:
    #                 pt = solutionPaths[i][iterateSolutionPaths[i]][0:2]
    #                 x, y = pt[0] * res, pt[1] * res
    #                 pygame.draw.circle(gameDisplay, pathColours[i], (int(x * res), int(y * res)),
    #                                    math.floor(3 * 1.5 * res))
    #                 pygame.time.delay(50)
    #                 iterateSolutionPaths[i] -= 1
    #                 if iterateSolutionPaths[i] == 0:
    #                     print("Robot " + str(i + 1) + " reached its goal")
    #                     iterateSolutionPaths[i] = -2
    #                 pygame.display.update()
    #
    # pygame.time.delay(4000)
    # pygame.quit()
    # print(subdimensionalExpansion(solutionPaths, nodesExplored, iterateSolutionPaths, scale, threshDistance, threshAngle,
    #                               radius + clearance))


def main():
    visualizeMStar()


if __name__ == "__main__":
    main()
