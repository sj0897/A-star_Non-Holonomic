#!/usr/bin/env python3

import sys
sys.path.append('/usr/lib/python3/dist-packages')
import os
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue 
import cv2
import argparse
import time
import math
import heapq
import rospy
from geometry_msgs.msg import Twist

def standardLine(p1, p2):
    # ax+by+d=0
    assert(p1!=p2),"point1 equals to point2, cannot form line"

    tangent_vector = (p2[0]-p1[0], p2[1]-p1[1])
    if (tangent_vector[0]==0):
        normal_vector = (1,0)
    elif (tangent_vector[1]==0): 
        normal_vector = (0,1)
    else:
        normal_vector = (1/(p2[0]-p1[0]), -1/(p2[1]-p1[1]))
    a, b = normal_vector
    norm = np.sqrt(pow(a, 2) + pow(b, 2))
    a, b = a / norm, b / norm 
    d = -(a * p1[0] + b * p1[1])
    return a, b, d


class Map:
    #mm
    width = 600 
    height = 200 
    occGrid = np.zeros((height+1, width+1))
    robot_radius = 5 + 5
    
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

    @classmethod
    def generateOccGrid(self, clearance): 
        
        Map.robot_radius = clearance + 18 #18 cm for turtle bot radius
        # boundary
        boundaryLine_1 = standardLine((0,0), (0,200))
        boundaryLine_2 = standardLine((0,200), (600,200))
        boundaryLine_3 = standardLine((600,200), (600,0))
        boundaryLine_4 = standardLine((0,0), (600,0))

        # upper rectangle
        upperRectangleLine_1 = standardLine((150, 200), (150, 75))
        upperRectangleLine_2 = standardLine((165, 200), (165, 75))
        upperRectangleLine_3 = standardLine((150, 75), (165, 75))

        # lower rectangle
        lowerRectangleLine_1 = standardLine((250, 125), (250, 0))
        lowerRectangleLine_2 = standardLine((265, 125), (265, 0))
        lowerRectangleLine_3 = standardLine((250, 125), (265, 125))

        # circle
        center = (400, 110)
        c_x, c_y = center
        c_r = 50


        # # hexagon
        # edge =  75
        # hexagonLine_1 = standardLine((235, 125 + edge/2), (235, 125 - edge/2))
        # hexagonLine_2 = standardLine((235, 125 + edge/2), (300, 125 + edge))
        # hexagonLine_3 = standardLine((300, 125 + edge), (365, 125 + edge/2))
        # hexagonLine_4 = standardLine((365, 125 + edge/2), (365, 125 - edge/2))
        # hexagonLine_5 = standardLine((300, 125 - edge), (365, 125 - edge/2))
        # hexagonLine_6 = standardLine((235, 125 - edge/2), (300, 125 - edge))

        rows, cols = Map.occGrid.shape 
        for i in range(0, rows):
            for j in range(0, cols): 
                # transform from top-left (0,0) to bottom-left (0,0)
                x = j
                y = rows - 1 - i

                # boundary with clearance
                if ((boundaryLine_1[0] * x + boundaryLine_1[1] * y + boundaryLine_1[2]) <=  Map.robot_radius or \
                    (boundaryLine_2[0] * x + boundaryLine_2[1] * y + boundaryLine_2[2]) >= -Map.robot_radius or \
                    (boundaryLine_3[0] * x + boundaryLine_3[1] * y + boundaryLine_3[2]) >= -Map.robot_radius or \
                    (boundaryLine_4[0] * x + boundaryLine_4[1] * y + boundaryLine_4[2]) <=  Map.robot_radius ): 
                    Map.occGrid[i, j]=2

                    # boundary
                    if ((boundaryLine_1[0] * x + boundaryLine_1[1] * y + boundaryLine_1[2]) <= 0 or \
                        (boundaryLine_2[0] * x + boundaryLine_2[1] * y + boundaryLine_2[2]) >= 0 or \
                        (boundaryLine_3[0] * x + boundaryLine_3[1] * y + boundaryLine_3[2]) >= 0 or \
                        (boundaryLine_4[0] * x + boundaryLine_4[1] * y + boundaryLine_4[2]) <= 0 ): 
                        Map.occGrid[i, j]=1

                # Rectangle with clearance
                if ((upperRectangleLine_1[0] * x + upperRectangleLine_1[1] * y + upperRectangleLine_1[2]) >= -Map.robot_radius and \
                    (upperRectangleLine_2[0] * x + upperRectangleLine_2[1] * y + upperRectangleLine_2[2]) <=  Map.robot_radius and \
                    (upperRectangleLine_3[0] * x + upperRectangleLine_3[1] * y + upperRectangleLine_3[2]) >= -Map.robot_radius): 
                    Map.occGrid[i, j]=2
                    # Rectangle
                    if ((upperRectangleLine_1[0] * x + upperRectangleLine_1[1] * y + upperRectangleLine_1[2]) >= 0 and \
                    (upperRectangleLine_2[0] * x + upperRectangleLine_2[1] * y + upperRectangleLine_2[2]) <=  0 and \
                    (upperRectangleLine_3[0] * x + upperRectangleLine_3[1] * y + upperRectangleLine_3[2]) >= 0): 
                        Map.occGrid[i, j]=1
                # Rectangle with clearance
                if ((lowerRectangleLine_1[0] * x + lowerRectangleLine_1[1] * y + lowerRectangleLine_1[2]) >= -Map.robot_radius and \
                    (lowerRectangleLine_2[0] * x + lowerRectangleLine_2[1] * y + lowerRectangleLine_2[2]) <=  Map.robot_radius and \
                    (lowerRectangleLine_3[0] * x + lowerRectangleLine_3[1] * y + lowerRectangleLine_3[2]) <= Map.robot_radius): 
                        Map.occGrid[i, j]=2
                        # Rectangle
                        if ((lowerRectangleLine_1[0] * x + lowerRectangleLine_1[1] * y + lowerRectangleLine_1[2]) >= 0 and \
                        (lowerRectangleLine_2[0] * x + lowerRectangleLine_2[1] * y + lowerRectangleLine_2[2]) <=  0 and \
                        (lowerRectangleLine_3[0] * x + lowerRectangleLine_3[1] * y + lowerRectangleLine_3[2]) <= 0): 
                            Map.occGrid[i, j]=1   

                if (pow((x-c_x), 2) + pow((y-c_y), 2)) <= pow(c_r+Map.robot_radius, 2): 
                    Map.occGrid[i, j]=2
                    if (pow((x-c_x), 2) + pow((y-c_y), 2)) <= pow(c_r, 2): 
                        Map.occGrid[i, j]=1
               

    @classmethod
    def isValid(self, pos): 
        rows, cols = Map.occGrid.shape 
        x, y, _ = pos
        j = x
        i = rows - 1 - y

        if  0<j<600 and 0<i<200:
         return Map.occGrid[i, j]==0
        else: 
            return False 


class Node:
    def __init__(self, pos=(0, 0, 0), cost2come = 0, cost2go = 0, parent=None, action = None, radius_c = 0): 
        self.pos = pos
        self.cost2come = cost2come
        self.cost2go = cost2go
        self.parent = parent
        self.action = action
        self. radius_c = radius_c        
    def __lt__(self, other):
        return self.cost2come + self.cost2go < other.cost2come + other.cost2go
        
    def __le__(self, other):
        return self.cost2come + self.cost2go <= other.cost2come + other.cost2go
    
def boundAngle(thetha, d_thetha = 0):
        result = thetha + d_thetha
        if result >= 360:
            result = result - 360
        elif result < 0:
            result = 360 + result
        
        return result

def actionCost(deltaThetha, R):
        cost = 0
        if deltaThetha == 0:
            cost = R
        else:
            cost = (deltaThetha / 360) * 2 * math.pi * R
            
        return cost

MAP_RESOLUTION_SCALE = 10
MAP_THRESOLD_REGION = int(0.5 * MAP_RESOLUTION_SCALE)
ANGLE_RESOLUTION = 30 # degree
MOBILE_ROBOT_RADIUS = 9 
ROBOT_RADIUS_INNER = 8 
TIME_INTERVAL = 0.5

class A_star: 
    def __init__(self, startPos, goalPos, rpm_1, rpm_2 ):
        self.openList = []
        self.closedList = set()
        self.closedListNodes = []
        
        self.forVisualization = []

        self.closedNodeMap = np.zeros((201 * MAP_RESOLUTION_SCALE, 
                                       601 * MAP_RESOLUTION_SCALE), np.uint8)

        self.actions = [[0,rpm_1],[rpm_1,0],[rpm_1,rpm_1],[0,rpm_2],[rpm_2,0],[rpm_2,rpm_2],[rpm_1,rpm_2],[rpm_2,rpm_1]]
        self.startPos = startPos
        self.goalPos = goalPos
             

    def addNode(self, node): 
        if node != None:
            isNodeSafe = Map.isValid(node.pos)
            
            if isNodeSafe:
                if not self.isNodeClosed(node):
                    heapq.heappush(self.openList, node)
                    self.forVisualization.append(node)

    def isNodeClosed(self, node): 
        # Transform x, y cart coord to w, h image coord
        rows, cols = Map.occGrid.shape 
        x, y, _ = node.pos
        j = x
        i = rows - 1 - y
        return self.closedNodeMap[i * MAP_RESOLUTION_SCALE, 
                                  j * MAP_RESOLUTION_SCALE] != 0

    def generateChildNodes(self, node): 

        action_sets = self.actions

        branches = []
        for action in action_sets:
            child = self.generateChild(node, action)
            if(child != None):
                branches.append(child)

        return branches
    
    def generateChild(self, node, action):
        
        x = node.pos[0]
        y = node.pos[1]
        thetha = math.radians(node.pos[2])
        
        objNode = None
        vL, vR = action
        
        if vL < 0 and vR < 0:
            pass # don't handle backward motion
        else:
            # forward motion, slight turn and sharp turns
            R = 0
            newX = 0
            newY = 0
            if vL != vR:
                # DIFFERENTIAL KINEMATICS
                R = abs(0.5 * ((vL + vR) / (vR - vL))) + ROBOT_RADIUS_INNER
                angle = (vR - vL) / (2 * ROBOT_RADIUS_INNER)                
                omega_dt = angle * TIME_INTERVAL
                
                # newThetha = thetha + omega_dt
                # newX = x + R * math.cos(newThetha)
                # newY = x + R * math.sin(newThetha)
                # newThetha = Utility.actionInDegree(math.degrees(newThetha))
                
                #JACOBIAN KINTEMATICS
                
                ICCx = x - R * math.sin(thetha)
                ICCy = y + R * math.cos(thetha)
                
                ICC_ORIGIN = np.array([[x - ICCx],
                                      [y - ICCy],
                                      [thetha]])
                
                ICC = np.array([[ICCx],
                               [ICCy],
                               [omega_dt]])
                
                ROT_ICC = np.array([[math.cos(omega_dt), -math.sin(omega_dt), 0],
                                   [math.sin(omega_dt), math.cos(omega_dt) , 0],
                                   [                 0,                  0 , 1]])
                
                new_pose = ROT_ICC @ ICC_ORIGIN + ICC
                newX = new_pose[0][0]
                newY = new_pose[1][0]
                newThetha = new_pose[2][0]
                newThetha = boundAngle(math.degrees(newThetha))
                
            else:
                R = vL * TIME_INTERVAL
                newX = x + R  * math.cos(thetha)
                newY = y + R * math.sin(thetha)
                newThetha = boundAngle(math.degrees(thetha))
            
        delt1aThetha = abs(newThetha - thetha)
    
        res = (int(newX), int(newY), thetha)
        
        if Map.isValid(res):
            # if res != self.coord:
                #calculating the cost2come by adding the parent and action cost2come
                action_cost = actionCost(delt1aThetha, R)
                cost2come = round( action_cost + node.cost2come, 3)
                objNode = Node(pos = res,cost2come = cost2come,cost2go = self.calculateCost2GO(res),parent=node, action= action, radius_c= R)
            
        return objNode


    def generatePath(self, node):
        path = []
        actions = []

        while(node.parent != None):
            path.append(node)
            actions.append(node.action)
            node = node.parent
        path.append(Node(pos = self.startPos))
        path.reverse()
        actions.reverse()

        print("Searched nodes: ", len(self.closedList))
        print("Solution steps: ", len(path))
        return self.forVisualization, path , actions

    def search(self):

        self.addNode(Node(pos = self.startPos,cost2come = 0,cost2go = self.calculateCost2GO(self.startPos)))

        while(self.openList):

            currNode = heapq.heappop(self.openList)     

            if self.isNodeClosed(currNode):
                continue


            self.closedList.add(currNode.pos)
            self.AddtoClosedNodeMap(currNode)

            if(self.isThisGoalNode(currNode.pos)):
                print("Goal Reached")
                return self.generatePath(currNode)

            branches = self.generateChildNodes(currNode)
            for child in branches:
                self.addNode(child)
        else:
            print("Search failed")
            sys.exit(1)

    def calculateCost2GO(self, pos):
        x,y, _ = pos
        x1,y1, _ = self.goalPos
        return round(math.sqrt((x1 - x)**2 + (y1 - y)**2))
    
    def AddtoClosedNodeMap(self, node):
        rows, cols = Map.occGrid.shape 
        x, y, _ = node.pos
        j = x
        i = rows - 1 - y
        matrix_x = int(i * MAP_RESOLUTION_SCALE - MAP_THRESOLD_REGION)
        matrix_y = int(j * MAP_RESOLUTION_SCALE - MAP_THRESOLD_REGION)
        self.closedNodeMap[matrix_x, matrix_y] = 1
        
        for counter_x in range(1, 11):
            for counter_y in range(1, 11):
                self.closedNodeMap[matrix_x + counter_x , 
                                   matrix_y + counter_y] = 1
                
    def isThisGoalNode(self, nodeToCheck):
        xcentre, ycentre, end_theta = self.goalPos
        x, y, node_theta = nodeToCheck
        in_goal = (x - xcentre)**2 + (y -ycentre)**2 - (1.5)**2 < 0

        return in_goal
    
def showOccGrid(occGrid):
    rows, cols = occGrid.shape
    color_map = np.zeros((rows, cols, 3))
    
    color_map[np.where(occGrid == 0)] = np.array([255, 255, 255])
    color_map[np.where(occGrid == 1)] = np.array([0, 0, 0])
    color_map[np.where(occGrid == 2)] = np.array([0, 0, 255])
    color_map[np.where(occGrid == 3)] = np.array([255, 0, 0])   
    color_map[np.where(occGrid == 4)] = np.array([0, 255, 0])  
    
    return color_map


def generateVideo(process , path , goal, occGrid):

    rows, cols = occGrid.shape
    fourcc = cv2.VideoWriter_fourcc('F','M','P','4')
    video = cv2.VideoWriter('TestCase_3.avi', fourcc, float(20), (601, 201))
    
    c_x, c_y, _ = goal
    for x in range(c_x-3, c_x+3): 
        for y in range(c_y-3, c_y+3): 
            if(pow((x-c_x), 2) + pow((y-c_y), 2)) <= pow(3, 2): 
                j = x
                i = rows - 1 - y
                occGrid[i, j]=4

    visualizationGrid = occGrid.copy()
    frame = showOccGrid(visualizationGrid)
    initialized = False
    for node in process:  
            if node.parent != None:
            # x, y to row col system
                action = node.action
            
                if action[0] == action[1]:
                    x, y, _ = node.parent.pos
                    j = x
                    i = rows - 1 - y

                    x, y, _ = node.pos
                    j1 = x
                    i1 = rows - 1 - y

                    start = (j,i)
                    end = (j1,i1)

                    cv2.line(frame,start,end,(255,0,0),1)    
                    video.write(np.uint8(frame))
                else :
                    pts = []
                    start = 0
                    end = 0

                    start_thetha = node.parent.pos[2]
                    end_thetha = node.pos[2]
                    r = node.radius_c

                    x, y, _ = node.parent.pos
                    j = x
                    i = rows - 1 - y

                    if start_thetha < end_thetha:
                        start = int(start_thetha)
                        end = int(end_thetha)

                        for i in range(start, end + 1, 1):
                            curve_x = (x + r * math.cos(math.radians(i)))
                            curve_y = (y + r * math.sin(math.radians(i)))
                            j1 = curve_x
                            i1 = rows - 1 - curve_y

                            start = (int(j),int(i))
                            end = (int(j1),int(i1)) 

                            cv2.line(frame,start,end,(255,0,0),1)

                            j = j1
                            i = i1
                            
                    else:
                        start = int(start_thetha)
                        end = int(end_thetha)
                        curve_x = (x + r * math.cos(math.radians(i)))
                        curve_y = (y + r * math.sin(math.radians(i)))
                        j1 = curve_x
                        i1 = rows - 1 - curve_y

                        start = (int(j),int(i))
                        end = (int(j1),int(i1))

                        cv2.line(frame,start,end,(255,0,0),1)

                        j = j1
                        i = i1
                    
                    # print(start, end)
                    
                        
                        # pts = np.array(pts, np.int32)
                        # pts = pts.reshape((-1, 1, 2))
                        # cv2.polylines(frame, [pts], False, (255,0,0))

    initialized = False                  
    # cv2.imshow("Map", frame)
    path.pop(0)
    # cv2.waitKey(0)
    
    for node in path:  
        if node.parent != None:
        # x, y to row col system
            action = node.action
        
            if action[0] == action[1]:
                x, y, _ = node.parent.pos
                j = x
                i = rows - 1 - y

                x, y, _ = node.pos
                j1 = x
                i1 = rows - 1 - y

                start = (j,i)
                end = (j1,i1)

                cv2.line(frame,start,end,(0,255,0),3)    
                video.write(np.uint8(frame))
            else :
                pts = []
                start = 0
                end = 0

                start_thetha = node.parent.pos[2]
                end_thetha = node.pos[2]
                r = node.radius_c

                x, y, _ = node.parent.pos
                j = x
                i = rows - 1 - y

                if start_thetha < end_thetha:
                    start = int(start_thetha)
                    end = int(end_thetha)

                    for i in range(start, end + 1, 1):
                        curve_x = (x + r * math.cos(math.radians(i)))
                        curve_y = (y + r * math.sin(math.radians(i)))
                        j1 = curve_x
                        i1 = rows - 1 - curve_y

                        start = (int(j),int(i))
                        end = (int(j1),int(i1)) 

                        cv2.line(frame,start,end,(0,255,0),3)

                        j = j1
                        i = i1
                        
                else:
                    start = int(start_thetha)
                    end = int(end_thetha)
                    curve_x = (x + r * math.cos(math.radians(i)))
                    curve_y = (y + r * math.sin(math.radians(i)))
                    j1 = curve_x
                    i1 = rows - 1 - curve_y

                    start = (int(j),int(i))
                    end = (int(j1),int(i1))

                    cv2.line(frame,start,end,(0,255,0),3)

                    j = j1
                    i = i1
    
        video.write(np.uint8(frame))
    cv2.imwrite("TestCase_3.jpg", frame)
    cv2.imshow("Map", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()        
    video.release()




def main():
    rospy.init_node('A_star', anonymous=True)

    if(len(sys.argv) > 1):
        start = (int(float(sys.argv[1])*100) + 50, int(float(sys.argv[2])*100) + 100, int(float(sys.argv[3])*180/3.14))
        goal= (int(sys.argv[4])+50, int(sys.argv[5])+100, 0)
        rpm_1 = int(sys.argv[6])
        rpm_2 = int(sys.argv[7])
        clearance = int(sys.argv[8])
    else:
        start = None
        goal = None
        rpm_1 = 10
        rpm_2 = 15
        clearance = 5
    Map.generateOccGrid(clearance)
    startTime = time.time()

    print(start)
    print(goal)
    # start , goal, rpm_1 , rpm_2  = getInputs() 
    graph = A_star(start, goal, rpm_1 , rpm_2)
    process, path , actions = graph.search()
    print(actions)
    intermediateTime = time.time()
    print("Algorithm Execution time:", intermediateTime - startTime, "seconds")
    generateVideo(process , path , goal, Map.occGrid)
    endTime = time.time()
    print("Rendering time:",endTime -  intermediateTime, "seconds")
    msg=Twist()
    initialized = False
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while not rospy.is_shutdown():
        if not initialized:
            for action in actions:
                wheel_radius = 0.033 # meters
                wheel_base = 0.16 # meters
                left_vel = action[0] * 6.28 * wheel_radius / 60
                right_vel = action[1] * 6.28 * wheel_radius / 60
                linear_vel = (left_vel + right_vel) / 2
                angular_vel = (right_vel - left_vel) / wheel_base

                msg.angular.z=angular_vel
                msg.linear.x= linear_vel
                #buff='my current time is %s" %rospy.get_time()
                pub.publish(msg)
                time.sleep(1)
            initialized = True
        msg.angular.z=0
        msg.linear.x= 0
        #buff='my current time is %s" %rospy.get_time()
        pub.publish(msg)



if __name__ == '__main__':
    main()

