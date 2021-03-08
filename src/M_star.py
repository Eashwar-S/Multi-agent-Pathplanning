import numpy as np
import heapq
import math
import time
from Environment import *


class Node:
    def __init__(self, state=None, cost=float('inf'), costToCome=float('inf'), parent=None):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.costToCome = costToCome


def pointInValidWorkspace(point, res, radiusClearance, scale):
    x, y = point

    # --------------------------------------------------------------------------------
    #                     Checking whether point inside obstacles
    # --------------------------------------------------------------------------------
    X = np.float32([8, 12.5, 12.5, 8]) * scale / res
    Y = np.float32([9, 9, 9.5, 9.5]) * scale / res
    ptInRectangle = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                    0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                    0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([10, 10.5, 10.5, 10]) * scale / res
    Y = np.float32([7, 7, 14.5, 14.5]) * scale / res
    ptInRectangle1 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([4, 4.25, 4.25, 4]) * scale / res
    Y = np.float32([8, 8, 10.5, 10.5]) * scale / res
    ptInRectangle2 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([1.5, 3, 3, 1.5]) * scale / res
    Y = np.float32([9, 9, 9.25, 9.25]) * scale / res
    ptInRectangle3 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([16, 16.25, 16.25, 16]) * scale / res
    Y = np.float32([8, 8, 10.5, 10.5]) * scale / res
    ptInRectangle4 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([17, 17.5, 17.5, 17]) * scale / res
    Y = np.float32([9, 9, 9.25, 9.25]) * scale / res
    ptInRectangle5 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) * scale / res
    Y = np.float32([3, 3, 3.25, 3.25]) * scale / res
    ptInRectangle6 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([10.15, 10.35, 10.35, 10.15]) * scale / res
    Y = np.float32([0.8, 0.8, 2.3, 2.3]) * scale / res
    ptInRectangle7 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) * scale / res
    Y = np.float32([15, 15, 15.25, 15.25]) * scale / res
    ptInRectangle8 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([10.15, 10.40, 10.40, 10.15]) * scale / res
    Y = np.float32([16, 16, 17.5, 17.5]) * scale / res
    ptInRectangle9 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    if ptInRectangle or ptInRectangle1 or ptInRectangle2 or ptInRectangle3 or ptInRectangle4 or \
            ptInRectangle5 or ptInRectangle6 or ptInRectangle7 or ptInRectangle8 or ptInRectangle9:
        return False
    return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, scale, r=1, radiusClearance=0):
    col = math.floor(800 / r)
    row = math.floor(800 / r)

    if newState[0] < 0 or newState[0] > col or newState[1] < 0 or newState[1] > row:
        return False
    return pointInValidWorkspace(newState[0:2], r, radiusClearance, scale)


# prints solution path
def printPath(node):
    l = []
    current = node
    while current:
        l.append(current.state)
        current = current.parent
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

        for theta in range(12):
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

                    newNode = Node(newState, newCost, newCostToCome, currentNode)
                    nodesExplored[s] = newNode

                    heapq.heappush(q, (newNode.cost, count, newNode))
                    count += 1
            else:
                if (nodesExplored[s].cost > currentNode.costToCome + distance([newState[0], newState[1]],
                                                                              [x, y]) + distance(
                    [newState[0], newState[1]], [gx, gy])):
                    nodesExplored[s].costToCome = currentNode.costToCome + distance([newState[0], newState[1]], [x, y])
                    nodesExplored[s].cost = nodesExplored[s].costToCome + distance([newState[0], newState[1]], [gx, gy])
                    nodesExplored[s].parent = currentNode

    return [False, None]  # checks whether next action is near an obstacle or ill defined


# TODO
def subdimensionalExpansion():
    pass


# def main():
#     print(pointInValidWorkspace((11.17, 16), 1, 0))
#     environment2D((11.17, 16))
#     # TODO
#     # Make Astar work for multiple robots
#
#
# if __name__ == "__main__":
#     main()


def triangleCoordinates(start, end, triangleSize=5):
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi / 2
    # print(math.atan2(start[1] - end[1], end[0] - start[0]))
    rad = math.pi / 180

    coordinateList = np.array([[end[0], end[1]],
                               [end[0] + triangleSize * math.sin(rotation - 165 * rad),
                                end[1] + triangleSize * math.cos(rotation - 165 * rad)],
                               [end[0] + triangleSize * math.sin(rotation + 165 * rad),
                                end[1] + triangleSize * math.cos(rotation + 165 * rad)]])

    return coordinateList


###################################################
#                  Parameters
###################################################
clearance = 3
radius = 0

stepSize = 11

# Start time of simulation
startTime = time.time()

# Step size of movement
threshDistance = stepSize

res = 1  # resolution of grid
scale = 40  # scale of grid

start = [[1 * scale, 16 * scale], [1 * scale, 1 * scale]]
goal = [[16 * scale, 1 * scale], [16 * scale, 16 * scale]]
start1 = 1 * scale
start2 = 16 * scale
goal1 = 16 * scale
goal2 = 1 * scale
s1 = 1 * scale
s2 = 1 * scale

g1 = 16 * scale
g2 = 16 * scale

# Angle between actions
threshAngle = 45

startOrientation = 0

pygame.init()

white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
orange = (255, 140, 0)
blue = (0, 0, 255)
purple = (75, 0, 130)
yellow = (255, 255, 0)
pathColours = [red, purple]
colors = [green, orange]
solutionPaths = []
size_x = 20
size_y = 20
gameDisplay = pygame.display.set_mode((size_x * scale, size_y * scale))
gameDisplay.fill(white)
############################################################
#                 Display Obstacles
############################################################
pygame.draw.rect(gameDisplay, black, [int(scale * 8), int(scale * 9), int(scale * 4.5), int(scale * 0.5)])  # plus
pygame.draw.rect(gameDisplay, black, [int(scale * 10), int(scale * 7), int(scale * 0.5), int(scale * 4.5)])  # plus
pygame.draw.rect(gameDisplay, black, [int(scale * 4), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
pygame.draw.rect(gameDisplay, black, [int(scale * 1.5), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
pygame.draw.rect(gameDisplay, black, [int(scale * 16), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
pygame.draw.rect(gameDisplay, black, [int(scale * 17), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 3), int(scale * 2.5), int(scale * 0.25)])  # -
pygame.draw.rect(gameDisplay, black, [int(scale * 10.15), int(scale * 0.8), int(scale * 0.25), int(scale * 1.5)])  # |
pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 15), int(scale * 2.5), int(scale * 0.25)])  # -
pygame.draw.rect(gameDisplay, black, [int(scale * 10.15), int(scale * 16), int(scale * 0.25), int(scale * 1.5)])  # |

############################################################
#          Draw Explored Nodes and solution path
############################################################
for i in range(len(start)):
    pygame.draw.circle(gameDisplay, red, start[i], 0.1 * scale)
    pygame.draw.circle(gameDisplay, red, goal[i], 0.1 * scale)

    # pygame.draw.circle(gameDisplay, red, (start1, start2), 0.1 * scale)
    # pygame.draw.circle(gameDisplay, red, (goal1, goal2), 0.1 * scale)

    nodesExplored = {}
    q = []
    startPosition = np.round((np.array(start[i])) / res)
    goalPosition = np.round((np.array(goal[i])) / res)
    if not isSafe(startPosition, scale, res, clearance + radius) or not isSafe(goalPosition, scale, res,
                                                                               clearance + radius):
        basicfont = pygame.font.SysFont(None, 48)
        text = basicfont.render('Start or goal configuration of robot ' + str(i + 1) + ' is not in a valid workspace',
                                True, (255, 0, 0), (255, 255, 255))
        textrect = text.get_rect()
        textrect.centerx = gameDisplay.get_rect().centerx
        textrect.centery = gameDisplay.get_rect().centery

        gameDisplay.blit(text, textrect)
        pygame.display.update()
        pygame.time.delay(2000)

    else:
        print('Exploring workspace for robot ' + str(i + 1))
        success, solution = Astar(q, startPosition, startOrientation, goalPosition, nodesExplored, scale,
                                  threshDistance,
                                  threshAngle, clearance + radius)

        #############################################
        #      Drawing
        #############################################
        if success:
            solutionPaths.append(solution)
            print('Optimal path found for robot ' + str(i + 1))
            endTime = time.time()
            print("Total time taken for exploring nodes " + str(endTime - startTime) + " seconds.")
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
                        # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
                        # pygame.draw.circle(gameDisplay,green,(int(x2),int(y2)),2)
                        triangle = triangleCoordinates([x2, y2], [x, y], 8)
                        pygame.draw.polygon(gameDisplay, colors[i],
                                            [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])
                        pygame.display.update()
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
            # basicfont = pygame.font.SysFont(None, 48)
            # text = basicfont.render('Path can\'t be generated', True, (255, 0, 0), (255, 255, 255))
            # textrect = text.get_rect()
            # textrect.centerx = gameDisplay.get_rect().centerx
            # textrect.centery = gameDisplay.get_rect().centery
            #
            # gameDisplay.blit(text, textrect)
            # pygame.display.update()
            # pygame.time.delay(2000)

iterateSolutionPaths = []
for i in range(len(solutionPaths)):
    if solutionPaths[i]:
        iterateSolutionPaths.append(len(solutionPaths[i]) - 1)
    else:
        iterateSolutionPaths.append(-1)

# draw solution path
while not all(ele == -2 for ele in iterateSolutionPaths):
    for i in range(len(start)):
        if iterateSolutionPaths[i] != -2:
            if iterateSolutionPaths[i] == -1:
                print("There is no Path forRobot " + str(i+1))
                iterateSolutionPaths[-i] = -2
            elif iterateSolutionPaths[i] >= 0:
                pt = solutionPaths[i][iterateSolutionPaths[i]][0:2]
                x, y = pt[0] * res, pt[1] * res
                pygame.draw.circle(gameDisplay, pathColours[i], (int(x * res), int(y * res)), math.floor(3 * 1.5 * res))
                pygame.time.delay(100)
                iterateSolutionPaths[i] -= 1
                if iterateSolutionPaths[i] == 0:
                    print("Robot " + str(i+1) + " reached its goal")
                    iterateSolutionPaths[i] = -2
                pygame.display.update()


pygame.time.delay(4000)
# pygame.display.update()
# pygame.time.delay(2000)
pygame.quit()
