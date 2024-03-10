# Caleb Myers
# ENPM661
# Project 2

# Welcome to my Project 2 point robot maze solver. The algorithm used was the Dijkstra Algorithm.


# #---------------------------------------------------------------------------# #
# #------------------------------*****STEP 1*****-----------------------------# #
# #----------*****Define all data structures and important info*****----------# #  

import copy
import numpy as np
import cv2 as cv2
from queue import PriorityQueue


obstacle_all = []
open_nodes_queue = PriorityQueue()
open_nodes = []
closed_nodes = []
visited_nodes = []
obstacle = []
new_node = []
path = []
solun = []  


goal = [450,10]
initial_position = [0,[485,1150]]
# #-------------------------------------------------------# #
# #--------------------*****STEP 2*****-------------------# #
# #----------*****Create Map with Obstacles*****----------# #

clearance = 5
frame_width = 1201
frame_height = 501
fps = 500
out = cv2.VideoWriter('cjm_proj2.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))
board = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
board[:,:] = (0,0,0) #Set all colors to black.
out.write(board)

#OBS 1

rect1_points = np.array([[100-clearance,400+clearance],[175+clearance,400+clearance],[175+clearance,0],[100-clearance,0]])
rect1 = cv2.fillPoly(board, [rect1_points], [255,150,150])

rect1_points = np.array([[100-(clearance-1),400+(clearance-1)],[175+(clearance-1),400+(clearance-1)],[175+(clearance-1),0],[100-(clearance-1),0]])
rect1 = cv2.fillPoly(board, [rect1_points], [250,150,0])

rect1_points = np.array([[100,400],[175,400],[175,0],[100,0]])
rect1 = cv2.fillPoly(board, [rect1_points], [0,0,255])

#OBS 2
rect2_points = np.array([[275-clearance,501],[350+clearance,501],[350+clearance,100-clearance],[275-clearance,100-clearance]])
rect2 = cv2.fillPoly(board, [rect2_points], [255,150,150])

rect2_points = np.array([[275-(clearance-1),501],[350+(clearance-1),501],[350+(clearance-1),100-(clearance-1)],[275-(clearance-1),100-(clearance-1)]])
rect2 = cv2.fillPoly(board, [rect2_points], [250,150,0])

rect2_points = np.array([[275,501],[350,501],[350,100],[275,100]])
rect2 = cv2.fillPoly(board, [rect2_points], [0,0,255])

#OBS 3 Find the points of the hexagon
hexagon_points = np.array([[650,400+clearance],[800+clearance,325+clearance],[800+clearance,175-clearance],[650,100-clearance],[501-clearance,175-clearance],[501-clearance,325+clearance]])
hexagon = cv2.fillPoly(board, [hexagon_points], [255,150,150])

hexagon_points = np.array([[650,400+(clearance-1)],[800+(clearance-1),325+(clearance-1)],[800+(clearance-1),175-(clearance-1)],[650,100-(clearance-1)],[501-(clearance-1),175-(clearance-1)],[501-(clearance-1),325+(clearance-1)]])
hexagon = cv2.fillPoly(board, [hexagon_points], [250,150,0])

hexagon_points = np.array([[650,400],[800,325],[800,175],[650,100],[501,175],[501,325]])
hexagon = cv2.fillPoly(board, [hexagon_points], [0,0,255])

# OBS 4
rect31_points = np.array([[900-clearance,125+clearance],[1020+clearance,125+clearance],[1020+clearance,50-clearance],[900-clearance,50-clearance]])
rect31 = cv2.fillPoly(board, [rect31_points], [255,150,150])
rect32_points = np.array([[900-clearance,450+clearance],[1020+clearance,450+clearance],[1020+clearance,375-clearance],[900-clearance,375-clearance]])
rect32 = cv2.fillPoly(board, [rect32_points], [255,150,150])
rect33_points = np.array([[1020-clearance,450+clearance],[1100+clearance,450+clearance],[1100+clearance,50-clearance],[1020-clearance,50-clearance]])
rect33 = cv2.fillPoly(board, [rect33_points], [255,150,150])

rect31_points = np.array([[900-(clearance-1),125+(clearance-1)],[1020+(clearance-1),125+(clearance-1)],[1020+(clearance-1),50-(clearance-1)],[900-(clearance-1),50-(clearance-1)]])
rect31 = cv2.fillPoly(board, [rect31_points], [255,150,0])
rect32_points = np.array([[900-(clearance-1),450+(clearance-1)],[1020+(clearance-1),450+(clearance-1)],[1020+(clearance-1),375-(clearance-1)],[900-(clearance-1),375-(clearance-1)]])
rect32 = cv2.fillPoly(board, [rect32_points], [255,150,0])
rect33_points = np.array([[1020-(clearance-1),450+(clearance-1)],[1100+(clearance-1),450+(clearance-1)],[1100+(clearance-1),50-(clearance-1)],[1020-(clearance-1),50-(clearance-1)]])
rect33 = cv2.fillPoly(board, [rect33_points], [255,150,0])

rect31_points = np.array([[900,125],[1020,125],[1020,50],[900,50]])
rect31 = cv2.fillPoly(board, [rect31_points], [0,0,255])
rect32_points = np.array([[900,450],[1020,450],[1020,375],[900,375]])
rect32 = cv2.fillPoly(board, [rect32_points], [0,0,255])
rect33_points = np.array([[1020,450],[1100,450],[1100,50],[1020,50]])
rect33 = cv2.fillPoly(board, [rect33_points], [0,0,255])



for i in range(501):
    for j in range(1201):
        if np.array_equal(board[i,j], [255,150,0]) or np.array_equal(board[i,j], [0,0,255]) or np.array_equal(board[i,j], [255,150,150]):
            obstacle.append([i,j])
            



# #-----------------------------------------------------------------------------# #
# #-------------------------------*****STEP 3*****------------------------------# #
# #----------*****Function to move the robot in any one direction*****----------# #

def move_left(robot_position):
    
    robot_position_l = copy.deepcopy(robot_position)
    
    if robot_position_l[1][1] - 1 >= 0:
        robot_position_l[1][1] = robot_position_l[1][1] - 1
        if robot_position_l[1] not in obstacle and robot_position_l[1] not in closed_nodes:
            robot_position_l[0] = 1 + robot_position_l[0]
            if robot_position_l[1] not in [node[1] for node in visited_nodes]:
                
                c_p = []
                c_p.append(robot_position_l)
                c_p.append(robot_position)
                path.append(c_p)
                
                visited_nodes.append(robot_position_l)
                open_nodes.append(robot_position_l)
                open_nodes_queue.put(robot_position_l)
            elif robot_position_l[1] in open_nodes[:][1] and robot_position_l[1] in path[:][0][1]:
                idx_open = open_nodes[:][1].index(robot_position_l[1])
                idx_c = path[:][0][1].index(robot_position_l[1])
                if robot_position_l[0] < open_nodes[idx_open][0]:
                    open_nodes[idx_open][0] = robot_position_l[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_l
                    
                

            
            
            
def move_right(robot_position):
    
    robot_position_r = copy.deepcopy(robot_position)
    
    if robot_position_r[1][1] + 1 <= 1200:
        robot_position_r[1][1] = robot_position_r[1][1] + 1
        if robot_position_r[1] not in obstacle and robot_position_r[1] not in closed_nodes:
            robot_position_r[0] = 1 + robot_position_r[0]
            if robot_position_r[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_r)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_r)
                open_nodes.append(robot_position_r)
                open_nodes_queue.put(robot_position_r)
            elif robot_position_r[1] in open_nodes[:][1] and robot_position_r[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_r[1])
                idx_c = path[:][0][1].index(robot_position_r[1])
                if robot_position_r[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_r[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_r
def move_up(robot_position):
    
    robot_position_u = copy.deepcopy(robot_position)
    
    if robot_position_u[1][0] + 1 <= 500:
        robot_position_u[1][0] = robot_position_u[1][0] + 1
        if robot_position_u[1] not in obstacle and robot_position_u[1] not in closed_nodes:
            robot_position_u[0] = 1 + robot_position_u[0]
            if robot_position_u[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_u)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_u)
                open_nodes.append(robot_position_u)
            elif robot_position_u[1] in open_nodes[:][1] and robot_position_u[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_u[1])
                idx_c = path[:][0][1].index(robot_position_u[1])
                open_nodes_queue.put(robot_position_u)
                if robot_position_u[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_u[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_u
def move_down(robot_position):
    
    robot_position_d = copy.deepcopy(robot_position)
    
    if robot_position_d[1][0] - 1 >= 0:cmyers17_...blem1.mp4
        robot_position_d[1][0] = robot_position_d[1][0] - 1
        if robot_position_d[1] not in obstacle and robot_position_d[1] not in closed_nodes:
            robot_position_d[0] = 1 + robot_position_d[0]
            if robot_position_d[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_d)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_d)
                open_nodes.append(robot_position_d)
                open_nodes_queue.put(robot_position_d)
            elif robot_position_d[1] in open_nodes[:][1] and robot_position_d[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_d[1])
                idx_c = path[:][0][1].index(robot_position_d[1])
                if robot_position_d[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_d[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_d
            
            
def move_up_left(robot_position):
    
    robot_position_ul = copy.deepcopy(robot_position)
    
    if robot_position_ul[1][0] + 1 <= 500 and robot_position_ul[1][1] - 1 >=0:
        robot_position_ul[1][1] = robot_position_ul[1][1] - 1
        robot_position_ul[1][0] = robot_position_ul[1][0] + 1
        if robot_position_ul[1] not in obstacle and robot_position_ul[1] not in closed_nodes:
            robot_position_ul[0] = 1.4 + robot_position_ul[0]
            if robot_position_ul[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_ul)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_ul)
                open_nodes.append(robot_position_ul)
                open_nodes_queue.put(robot_position_ul)
            elif robot_position_ul[1] in open_nodes[:][1] and robot_position_ul[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_ul[1])
                idx_c = path[:][0][1].index(robot_position_ul[1])
                if robot_position_ul[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_ul[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_ul
            
            
def move_up_right(robot_position):

    robot_position_ur = copy.deepcopy(robot_position)
    
    if robot_position_ur[1][1] + 1 <= 1200 and robot_position_ur[1][0] + 1 <= 500:
        robot_position_ur[1][0] = robot_position_ur[1][0] + 1
        robot_position_ur[1][1] = robot_position_ur[1][1] + 1
        if robot_position_ur[1] not in obstacle and robot_position_ur[1] not in closed_nodes:
            robot_position_ur[0] = 1.4 + robot_position_ur[0]
            if robot_position_ur[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_ur)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_ur)
                open_nodes.append(robot_position_ur)
                open_nodes_queue.put(robot_position_ur)
            elif robot_position_ur[1] in open_nodes[:][1] and robot_position_ur[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_ur[1])
                idx_c = path[:][0][1].index(robot_position_ur[1])
                if robot_position_ur[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_ur[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_ur

def move_down_left(robot_position):
    
    robot_position_dl = copy.deepcopy(robot_position)
    
    if robot_position_dl[1][1] - 1 >= 0 and robot_position_dl[1][0] - 1 >= 0:
        robot_position_dl[1][0] = robot_position_dl[1][0] - 1
        robot_position_dl[1][1] = robot_position_dl[1][1] - 1
        if robot_position_dl[1] not in obstacle and robot_position_dl[1] not in closed_nodes:
            robot_position_dl[0] = 1.4 + robot_position_dl[0]
            if robot_position_dl[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_dl)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_dl)
                open_nodes.append(robot_position_dl)
                open_nodes_queue.put(robot_position_dl)
            elif robot_position_dl[1] in open_nodes[:][1] and robot_position_dl[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_dl[1])
                idx_c = path[:][0][1].index(robot_position_dl[1])
                if robot_position_dl[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_dl[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_dl
def move_down_right(robot_position):
    
    robot_position_dr = copy.deepcopy(robot_position)
    
    if robot_position_dr[1][0] - 1 >= 0 and robot_position_dr[1][1] + 1 <= 1200:
        robot_position_dr[1][0] = robot_position_dr[1][0] - 1
        robot_position_dr[1][1] = robot_position_dr[1][1] + 1
        if robot_position_dr[1] not in obstacle and robot_position_dr[1] not in closed_nodes:
            robot_position_dr[0] = 1.4 + robot_position_dr[0]
            if robot_position_dr[1] not in [node[1] for node in visited_nodes]:
                c_p = []
                c_p.append(robot_position_dr)
                c_p.append(robot_position)
                path.append(c_p)
                visited_nodes.append(robot_position_dr)
                open_nodes.append(robot_position_dr)
                open_nodes_queue.put(robot_position_dr)
            elif robot_position_dr[1] in open_nodes[:][1] and robot_position_dr[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_dr[1])
                idx_c = path[:][0][1].index(robot_position_dr[1])
                if robot_position_dr[0] < open_nodes[idx][0]:
                    open_nodes[idx][0] = robot_position_dr[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_dr
                    
                    
                    
# #-------------------------------------------------# #
# #-----------------*****STEP 4*****----------------# #
# #----------*****BFS Search Function*****----------# #

def generate_path(initial_state, goal_state, path):
    #Generates the path
    final_path = []
    final_path.append(goal_state)                               #Append the final node state
    
    # This section solves for the path from initial state to goal state                                                    
    for i in range(len(path)):
        for x in range(len(path)):
            if final_path[i] == path[x][0][1]:                     #path[x][0] is the child node state collected with the respective parent node state
                final_path.append(path[x][1][1])                   #if the child node is equal to the prev node, save the parent node.
        if final_path[i] == initial_state[1]:                      #Once the final path index i is equal to the initial state end the backwards search
            break
    for y in range(len(final_path)):                            #reverses the final_path
        solun.append(final_path[len(final_path)-(y+1)])
        
    return solun


# #-------------------------------------------------# #
# #-----------------*****STEP 4*****----------------# #
# #----------*****BFS Search Function*****----------# #

def dijkstra_alg(robot_position):
    move_left(robot_position)
    move_right(robot_position)
    move_up(robot_position)
    move_down(robot_position)
    move_up_left(robot_position)
    move_up_right(robot_position)
    move_down_left(robot_position)
    move_down_right(robot_position)
    

# #-------------------------------------------------# #
# #-----------------*****STEP 4*****----------------# #
# #----------*****BFS Search Function*****----------# #

def finished(node):
    if node == goal:   #If the node_state popped out of unvisited nodes is the same as the goal, exit the while loop.
        finished = 0
        return finished
    else:
        finished = 1
        return finished

    
# #---------------------------------------# #
# #-------------*****STEP 8*****----------# #
# #----------*****Main Code*****----------# #


open_nodes.append(initial_position)
open_nodes_queue.get(initial_position)
visited_nodes.append(initial_position)
done = 1

while done == 1:
    
    # min_value = min(sublist[0] for sublist in open_nodes)
    # idx = 0
    # while(True):
    #     if open_nodes[idx][0] == min_value:
    #         break
    #     else:
    #         idx += 1
    
    robot = open_nodes_queue.get()
    robot_position = robot[1]
    open_nodes.remove(robot)
    robot_position[0] = round(robot_position[0],1)
    print(robot_position)
    done = finished(robot_position[1])
    
    dijkstra_alg(robot_position)
    closed_nodes.append(robot_position)
    
    board[robot_position[1][0],robot_position[1][1]] = (255,255,255)
    board[initial_position[1][0],initial_position[1][1]] = (0,255,0)
    board[goal[0],goal[1]] = (0,255,0)
    out.write(board)
    print(idx)
    

    
    
generate_path(initial_position, goal, path)

for x in range(len(solun)):
    board[solun[x][0],solun[x][1]] = (0,255,0)
    out.write(board)
    out.write(board)
    out.write(board)

print('\n\n\n')
print(solun)
print('SOLUT FOUND')


for z in range(200):
    out.write(board)
out.release()