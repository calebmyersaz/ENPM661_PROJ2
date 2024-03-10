# Caleb Myers
# ENPM661
# Project 2

# Welcome to my Project 2 point robot maze solver. The algorithm used was the Dijkstra Algorithm.


# #---------------------------------------------------------------------------# #
# #------------------------------*****STEP 1*****-----------------------------# #
# #----------*****Define all data structures and important info*****----------# #  

#import copy,np, and cv2
import copy
import numpy as np
import cv2 as cv2
import time

#initialize lists
obstacle_all = []
open_nodes = []
closed_nodes = []
visited_nodes = []
obstacle = []
new_node = []
path = []
solun = []


initial_position = [0,[0,0]]
goal = [19,20]


# #-------------------------------------------------------# #
# #--------------------*****STEP 2*****-------------------# #
# #----------*****Create Map with Obstacles*****----------# #

# Map is scaled down a factor of 5 to improve search time. (Approved by Professor Monfareid) 
clearance = 1 #clearance of 1 is 5 mm

#Use open CV to create a video
frame_width = 241
frame_height = 101
fps = 120
out = cv2.VideoWriter('dijsktra_caleb_myers.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))
board = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
board[:,:] = (0,0,0) #Set all colors to black.
out.write(board)






#OBS 1 (The first Rectangle)

#Prints a blue outline for the clearance
rect1_points = np.array([[20-clearance,80+clearance],[35+clearance,80+clearance],[35+clearance,0],[20-clearance,0]])
rect1 = cv2.fillPoly(board, [rect1_points], [255,0,0])

#Prints the red obstacle 
rect1_points = np.array([[20,80],[35,80],[35,0],[20,0]])
rect1 = cv2.fillPoly(board, [rect1_points], [0,0,255])

#OBS 2 (The second Rectangle)

#Prints a blue outline for the clearance
rect2_points = np.array([[55-clearance,320+clearance],[70+clearance,320+clearance],[70+clearance,20-clearance],[55-clearance,20-clearance]])
rect2 = cv2.fillPoly(board, [rect2_points], [255,0,0])

#Prints the red obstacle 
rect2_points = np.array([[55,100],[70,100],[70,20],[55,20]])
rect2 = cv2.fillPoly(board, [rect2_points], [0,0,255])

#OBS 3 Find the points of the hexagon  
#half plane equations were used to get the corners for the hexagon

#Prints a blue outline for the clearance
hexagon_points = np.array([[130,80+clearance],[160+clearance,65+clearance],[160+clearance,35-clearance],[130,20-clearance],[100-clearance,35-clearance],[100-clearance,65+clearance]])
hexagon = cv2.fillPoly(board, [hexagon_points], [255,0,0])

#Prints the red obstacle 
hexagon_points = np.array([[130,80],[160,65],[160,35],[130,20],[100,35],[100,65]])
hexagon = cv2.fillPoly(board, [hexagon_points], [0,0,255])

# OBS 4 (The last shape)

#Prints a blue outline for the clearance
rect31_points = np.array([[180-clearance,25+clearance],[204+clearance,25+clearance],[204+clearance,10-clearance],[180-clearance,10-clearance]])
rect31 = cv2.fillPoly(board, [rect31_points], [255,0,0])
rect32_points = np.array([[180-clearance,90+clearance],[204+clearance,90+clearance],[204+clearance,75-clearance],[180-clearance,75-clearance]])
rect32 = cv2.fillPoly(board, [rect32_points], [255,0,0])
rect33_points = np.array([[204-clearance,90+clearance],[220+clearance,90+clearance],[220+clearance,10-clearance],[204-clearance,10-clearance]])
rect33 = cv2.fillPoly(board, [rect33_points], [255,0,0])

#Prints the red obstacle 
rect31_points = np.array([[180,25],[204,25],[204,10],[180,10]])
rect31 = cv2.fillPoly(board, [rect31_points], [0,0,255])
rect32_points = np.array([[180,90],[204,90],[204,75],[180,75]])
rect32 = cv2.fillPoly(board, [rect32_points], [0,0,255])
rect33_points = np.array([[204,90],[220,90],[220,10],[204,10]])
rect33 = cv2.fillPoly(board, [rect33_points], [0,0,255])


#This appends all the obstacle pixels into the obstacle list
for i in range(101):
    for j in range(241):
        if np.array_equal(board[i,j], [255,0,0]) or np.array_equal(board[i,j], [0,0,255]) or np.array_equal(board[i,j], [255,150,150]):
            obstacle.append([i,j])
            
out.write(board)
# #-----------------------------------------------------------------------------# #
# #-------------------------------*****STEP 3*****------------------------------# #
# #----------*****Function to move the robot in any one direction*****----------# #


# #----------*****Function to move the robot left*****----------# #
def move_left(robot_position):
    
    #create a copy of the robot position
    robot_position_l = copy.deepcopy(robot_position)
    
    if robot_position_l[1][1] - 1 >= 0: #if moving left is still in the map
        robot_position_l[1][1] = robot_position_l[1][1] - 1
        if robot_position_l[1] not in obstacle and robot_position_l[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_l[0] = 5 + robot_position_l[0]  #adds 5 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_l[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                
                c_p = []
                c_p.append(robot_position_l)
                c_p.append(robot_position)
                path.append(c_p)
                
                #append the new node
                visited_nodes.append(robot_position_l)
                open_nodes.append(robot_position_l)
            elif robot_position_l[1] in open_nodes[:][1] and robot_position_l[1] in path[:][0][1]:
                idx_open = open_nodes[:][1].index(robot_position_l[1])
                idx_c = path[:][0][1].index(robot_position_l[1])
                if robot_position_l[0] < open_nodes[idx_open][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx_open][0] = robot_position_l[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_l
                    
                

            
            

# #----------*****Function to move the robot right*****----------# #
def move_right(robot_position):
    
    #create a copy of the robot position
    robot_position_r = copy.deepcopy(robot_position)
    
    if robot_position_r[1][1] + 1 <= 240: #if moving right is still in the map
        robot_position_r[1][1] = robot_position_r[1][1] + 1
        if robot_position_r[1] not in obstacle and robot_position_r[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_r[0] = 5 + robot_position_r[0] #adds 5 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_r[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_r)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_r)
                open_nodes.append(robot_position_r)
            elif robot_position_r[1] in open_nodes[:][1] and robot_position_r[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_r[1])
                idx_c = path[:][0][1].index(robot_position_r[1])
                if robot_position_r[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_r[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_r
                    
                    
# #----------*****Function to move the robot up*****----------# #
def move_up(robot_position):
    
    #create a copy of the robot position
    robot_position_u = copy.deepcopy(robot_position)
    
    if robot_position_u[1][0] + 1 <= 100: #if moving up is still in the map
        robot_position_u[1][0] = robot_position_u[1][0] + 1
        if robot_position_u[1] not in obstacle and robot_position_u[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_u[0] = 5 + robot_position_u[0] #adds 5 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_u[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_u)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_u)
                open_nodes.append(robot_position_u)
            elif robot_position_u[1] in open_nodes[:][1] and robot_position_u[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_u[1])
                idx_c = path[:][0][1].index(robot_position_u[1])
                if robot_position_u[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_u[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_u
                    
                    
# #----------*****Function to move the robot down*****----------# #
def move_down(robot_position):
    
    #create a copy of the robot position
    robot_position_d = copy.deepcopy(robot_position)
    
    if robot_position_d[1][0] - 1 >= 0: #if moving down is still in the map
        robot_position_d[1][0] = robot_position_d[1][0] - 1
        if robot_position_d[1] not in obstacle and robot_position_d[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_d[0] = 5 + robot_position_d[0] #adds 5 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_d[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_d)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_d)
                open_nodes.append(robot_position_d)
            elif robot_position_d[1] in open_nodes[:][1] and robot_position_d[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_d[1])
                idx_c = path[:][0][1].index(robot_position_d[1])
                if robot_position_d[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_d[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_d
            

# #----------*****Function to move the robot up-left*****----------# #
def move_up_left(robot_position):
    
    #create a copy of the robot position
    robot_position_ul = copy.deepcopy(robot_position)
    
    if robot_position_ul[1][0] + 1 <= 100 and robot_position_ul[1][1] - 1 >=0: #if moving up-left is still in the map
        robot_position_ul[1][1] = robot_position_ul[1][1] - 1
        robot_position_ul[1][0] = robot_position_ul[1][0] + 1
        if robot_position_ul[1] not in obstacle and robot_position_ul[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_ul[0] = 7.07 + robot_position_ul[0] #adds 7.07 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_ul[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_ul)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_ul)
                open_nodes.append(robot_position_ul)
            elif robot_position_ul[1] in open_nodes[:][1] and robot_position_ul[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_ul[1])
                idx_c = path[:][0][1].index(robot_position_ul[1])
                if robot_position_ul[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_ul[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_ul
            
            
# #----------*****Function to move the robot up-right*****----------# #
def move_up_right(robot_position):

    #create a copy of the robot position
    robot_position_ur = copy.deepcopy(robot_position)
    
    if robot_position_ur[1][1] + 1 <= 240 and robot_position_ur[1][0] + 1 <= 100: #if moving up-right is still in the map
        robot_position_ur[1][0] = robot_position_ur[1][0] + 1
        robot_position_ur[1][1] = robot_position_ur[1][1] + 1
        if robot_position_ur[1] not in obstacle and robot_position_ur[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_ur[0] = 7.07 + robot_position_ur[0] #adds 7.07 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_ur[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_ur)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_ur)
                open_nodes.append(robot_position_ur)
                
            elif robot_position_ur[1] in open_nodes[:][1] and robot_position_ur[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_ur[1])
                idx_c = path[:][0][1].index(robot_position_ur[1])
                if robot_position_ur[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_ur[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_ur

# #----------*****Function to move the robot down-left*****----------# #
def move_down_left(robot_position):
    
    #create a copy of the robot position
    robot_position_dl = copy.deepcopy(robot_position)
    
    if robot_position_dl[1][1] - 1 >= 0 and robot_position_dl[1][0] - 1 >= 0:   #if moving down-left is still in the map
        robot_position_dl[1][0] = robot_position_dl[1][0] - 1
        robot_position_dl[1][1] = robot_position_dl[1][1] - 1
        if robot_position_dl[1] not in obstacle and robot_position_dl[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_dl[0] = 7.07 + robot_position_dl[0] #adds 7.07 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_dl[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_dl)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_dl)
                open_nodes.append(robot_position_dl)
            elif robot_position_dl[1] in open_nodes[:][1] and robot_position_dl[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_dl[1])
                idx_c = path[:][0][1].index(robot_position_dl[1])
                if robot_position_dl[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_dl[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_dl
                    
                    
# #----------*****Function to move the robot down-right*****----------# #
def move_down_right(robot_position):
    
    #create a copy of the robot position
    robot_position_dr = copy.deepcopy(robot_position)
    
    if robot_position_dr[1][0] - 1 >= 0 and robot_position_dr[1][1] + 1 <= 240: #if moving down-right is still in the map
        robot_position_dr[1][0] = robot_position_dr[1][0] - 1
        robot_position_dr[1][1] = robot_position_dr[1][1] + 1
        if robot_position_dr[1] not in obstacle and robot_position_dr[1] not in closed_nodes: #if the new node has not been visited and is not in an obstacle
            robot_position_dr[0] = 7.07 + robot_position_dr[0] #adds 7.07 to the cost to come because each pixel movement is scaled down by a factor of 5
            if robot_position_dr[1] not in [node[1] for node in visited_nodes]: #if the node is new
                
                #creates the child parent node
                
                c_p = []
                c_p.append(robot_position_dr)
                c_p.append(robot_position)
                path.append(c_p)
                #append the new node
                visited_nodes.append(robot_position_dr)
                open_nodes.append(robot_position_dr)
            elif robot_position_dr[1] in open_nodes[:][1] and robot_position_dr[1] in path[:][0][1]:
                idx = open_nodes[:][1].index(robot_position_dr[1])
                idx_c = path[:][0][1].index(robot_position_dr[1])
                if robot_position_dr[0] < open_nodes[idx][0]: #if the cost to come is lower than that of the saved node, update the parent and the c2c
                    open_nodes[idx][0] = robot_position_dr[0]
                    path[idx_c][1] = robot_position
                    path[idx_c][0] = robot_position_dr
                    
                    
                    
# #---------------------------------------------------------------------------------# #
# #-------------------------------*****STEP 4*****----------------------------------# #
# #----------*****Generate the path from initial position to the goal*****----------# #

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


# #------------------------------------------------------# #
# #--------------------*****STEP 5*****------------------# #
# #----------*****Dijsktra Search Function*****----------# #

def dijkstra_alg(robot_position):
    
    #Move the node in each direction.
    #Each moving function already updates the parents nodes and c2c and updates the node lists.
    move_left(robot_position)
    move_right(robot_position)
    move_up(robot_position)
    move_down(robot_position)
    move_up_left(robot_position)
    move_up_right(robot_position)
    move_down_left(robot_position)
    move_down_right(robot_position)
    

# #--------------------------------------------------------------------# #
# #----------------------------*****STEP 6*****------------------------# #
# #----------*****Check if current node is the goal node*****----------# #

def finished(node):
    if node == goal:   #If the node_state popped out of open nodes is the same as the goal, exit the while loop.
        finished = 0
        return finished
    else:
        finished = 1
        return finished

    

    
# #---------------------------------------# #
# #-------------*****STEP 7*****----------# #
# #----------*****Main Code*****----------# #



#append the starting node into the open node list
open_nodes.append(initial_position)
visited_nodes.append(initial_position)
done = 1

start = time.time()
while done == 1:
    
    min_value = min(sublist[0] for sublist in open_nodes) #finds the min cost to come
    idx = 0
    while(True):
        if open_nodes[idx][0] == min_value: #gets the node index of the lowest cost to come
            break
        else:
            idx += 1
    
    #removes the lowest cost to come
    robot_position = open_nodes[idx]
    open_nodes.remove(open_nodes[idx])
    robot_position[0] = round(robot_position[0],2)
    print(robot_position)
    done = finished(robot_position[1]) # checks if the new node is the solution
    
    #perform the dijkstra alg
    dijkstra_alg(robot_position)
    closed_nodes.append(robot_position)
    
    #color the new searched node white
    board[robot_position[1][0],robot_position[1][1]] = (255,255,255)
    
    #color the start and goal node green
    board[initial_position[1][0],initial_position[1][1]] = (0,255,0)
    board[goal[0],goal[1]] = (0,255,0)
    
    #save the new image frame
    out.write(board)

end = time.time()

print('Runtime: ', round(end-start,3), 's')
print('Cost from initial position to the goal is: ', round(robot_position[0],1), 'mm')

    

    
# find the path from start node to goal node and paint the path green
generate_path(initial_position, goal, path)
for x in range(len(solun)):
    board[solun[x][0],solun[x][1]] = (0,255,0)
    out.write(board)
    out.write(board)
    out.write(board)
    
print('\n\n\n')
# print(solun)
print('SOLUT FOUND')
# print(len(visited_nodes))

for z in range(200):
    out.write(board)
    
out.release() #release the video
