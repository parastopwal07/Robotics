#this almost has a similar logic as dijkstra so only the new logic has been commented
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
import heapq #we use priority queues here so we need heapq to managae the queues

def valid_node(node, size_of_grid):
    if node[0] < 0 or node[0] >= size_of_grid:
        return False
    if node[1] < 0 or node[1] >= size_of_grid:
        return False
    return True

def heuristic(p1, p2): #Manhattan distance
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) #definition of manhattan distance. This is different from eucledian distance

def up(node):
    return (node[0] - 1, node[1])

def down(node):
    return (node[0] + 1, node[1])

def left(node):
    return (node[0], node[1] - 1)

def right(node):
    return (node[0], node[1] + 1)

def astar(initial_node, desired_node, obstacles):
    size_of_floor = obstacles.shape[0]
    
    # Define neighbor directions (up, down, left, and right only)
    directions = [up, down, left, right]
    
    open_set = [] #list to be used as prority queue that stores tuple (cost(from start to current), node)
    heapq.heappush(open_set, (0, initial_node)) #pushes initial node
    
    g_score = np.ones([size_of_floor, size_of_floor]) * np.inf #stores cost of each node from the start node and sets it to infinity
    g_score[initial_node[0], initial_node[1]] = 0 #sets value for initial node
    came_from = {} #initialises dictionary to store the node from which new node came from
    
    while open_set:
        _, current_node = heapq.heappop(open_set) #_ to ignore the cost and just store the node in current_node
        
        if current_node == desired_node:
            path = [current_node] #store in path list
            while current_node in came_from:
                current_node = came_from[current_node] #RHS gives the previous node that current_node came from and then updates current_node to its previous_node until it reaches initial node
                path.append(current_node)
            return list(reversed(path))
        
        for direction in directions:
            potential_node = direction(current_node) #takes in current_node and then goes through each direction to give potential_node
            if valid_node(potential_node, size_of_floor) and obstacles[potential_node[0], potential_node[1]] == 0: #checks obstacles and validity of the potential node
                tentative_g_score = g_score[current_node[0], current_node[1]] + 1  # g-score is total cost to reach current from start node. Take 1 for simplicity

                if tentative_g_score < g_score[potential_node[0], potential_node[1]]: #if calculated gscore is less than the already stored then update
                    came_from[potential_node] = current_node
                    g_score[potential_node[0], potential_node[1]] = tentative_g_score
                    f_score = tentative_g_score + heuristic(potential_node, desired_node) #fscore is sum of gscore and heuristic function
                    heapq.heappush(open_set, (f_score, potential_node)) #store in open_set
    
    return [] #return empty after open_set is empty

obstacles = np.array([[0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,1,0,0,0,0,0],
                      [0,0,0,1,0,1,0,0,0,0],
                      [1,1,1,1,0,1,1,0,0,0],
                      [0,0,0,1,0,1,0,0,0,0],
                      [0,0,0,0,0,1,1,0,0,0],
                      [0,0,0,0,0,1,0,0,0,0],
                      [0,0,0,0,1,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0]], dtype=float)

start = (0, 0)
goal = (3, 4)

path = astar(start, goal, obstacles)
print(path)

colors = ['white', 'black']
cmap = ListedColormap(colors)

plt.imshow(obstacles, cmap=cmap, interpolation='nearest')
plt.draw()
plt.pause(2)
colors.append('green')
cmap = ListedColormap(colors)
for node in path:
    if obstacles[node[0], node[1]] == 1:
        continue
    obstacles[node[0], node[1]] = 2
    plt.imshow(obstacles, cmap=cmap, interpolation='nearest')
    plt.draw()
    plt.pause(0.1)

time.sleep(3)