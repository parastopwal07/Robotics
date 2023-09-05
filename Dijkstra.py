import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time

def valid_node(node, size_of_grid):   #Checks if node is within the grid boundaries
    if node[0] < 0 or node[0] >= size_of_grid: #node is a tuple with 0 index representing row and 1 as column
        return False
    if node[1] < 0 or node[1] >= size_of_grid:
        return False
    return True

def up(node):
    return (node[0]-1,node[1]) #as y axis 0 is at the top and increases down

def down(node):
    return (node[0]+1,node[1])

def left(node):
    return (node[0],node[1]-1)

def right(node):
    return (node[0],node[1]+1)

def backtrack(initial_node, desired_node, distances): #backtracks from the desired to the initial node after dijkstra function is completed
    path = [desired_node] #storing desired node in path list

    size_of_grid = distances.shape[0] #finding size of the grid assuming it is square. shape is used to get the size of any grid

    while True: #infinite loop
        potential_distances = [] #creating lists to store distance and nodes to compare and find the least later
        potential_nodes = []

        directions = [up,down,left,right] #available options to move

        for direction in directions:
            node = direction(path[-1]) #for all the nodes it calculates all the possible neighboring nodes
            if valid_node(node, size_of_grid):
                potential_nodes.append(node)
                potential_distances.append(distances[node[0],node[1]]) #stores entry from distances at node[0] row and node[1] column 

        least_distance_index = np.argmin(potential_distances) #gives node with min distance from current node
        path.append(potential_nodes[least_distance_index]) #stores it in the path list

        if path[-1][0] == initial_node[0] and path[-1][1] == initial_node[1]: #checks if the latest additon to path is equal to inital node
            break

    return list(reversed(path)) #reverses the path to gives nodes from initital to desired node

def dijkstra(initial_node, desired_node, obstacles):

    obstacles = obstacles.copy() #copies obstacle array to make it store the cost (distance) basically cost heuristic array
    obstacles *= 1000 #increasing the obstacle value in the array by 1000 times so that the algorithm avoids the path as obstacle free path will remain 0
    obstacles += np.ones(obstacles.shape) #increases value of existing obstacles array by 1
    obstacles[initial_node[0],initial_node[1]] = 0 #making the initial and desired node obstacle free
    obstacles[desired_node[0],desired_node[1]] = 0
    size_of_floor = obstacles.shape[0] #stores the size of obstacles 

    visited = np.zeros([size_of_floor,size_of_floor]) #visited array with all 0 entries

    distances = np.ones([size_of_floor,size_of_floor]) * np.inf #again distances array with all entries infinity (as 1 is multiplied with infinity)
    distances[initial_node[0],initial_node[1]] = 0 #except the initial node which is initialised to 0

    # starting the algorithm
    current_node = [initial_node[0], initial_node[1]] #setting the current node
    while True: #infinite loop
        directions = [up, down, left, right]
        for direction in directions:
            potential_node = direction(current_node)
            if valid_node(potential_node, size_of_floor): 
                if not visited[potential_node[0],potential_node[1]]: # check if we have visited this node before
                    distance = distances[current_node[0], current_node[1]] + obstacles[potential_node[0],potential_node[1]] #calculated distance is stored 
                    #as sum of distance of current from initial and the cost of current to neightboring
                    if distance < distances[potential_node[0],potential_node[1]]: #checks if the calculated distance and distance already stored
                        distances[potential_node[0],potential_node[1]] = distance #if yes update the distances array

        visited[current_node[0],current_node[1]] = True #updated the value to 1 as it is visited

        t=distances.copy()
        t[np.where(visited)]=np.inf #sets visited nodes to infinity
        node_index = np.argmin(t) #stores the min distance node to node_index. This is 1D array

        node_row = node_index//size_of_floor #converting 1D to 2D array using standard method that is division and modulo
        node_col = node_index%size_of_floor

        current_node = (node_row, node_col) #updating the current node
        if current_node[0] == desired_node[0] and current_node[1]==desired_node[1]: #if desired reached then break
            break

    return backtrack(initial_node,desired_node,distances) #return the reversed path

obstacles = np.array([[0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,1,0,0,0,0,0],
                      [0,0,0,1,0,1,0,0,0,0],
                      [1,1,1,1,0,1,1,0,0,0],
                      [0,0,0,1,0,1,0,0,0,0],
                      [0,0,0,0,0,1,1,0,0,0],
                      [0,0,0,0,0,1,0,0,0,0],
                      [0,0,0,0,1,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0]], dtype=float) #assigning type of entries as float using dtype to help store values more preicisely

start =(0,0)
goal=(3,4)

path = dijkstra(start,goal,obstacles)
print(path)

colors = ['white', 'black'] #list of given colors
cmap = ListedColormap(colors) #colormap or cmap is used to assign a color to an integer value for representation

plt.imshow(obstacles, cmap=cmap, interpolation='nearest') #drawing initial state and obstacles with solid color (solid due to interpolation as nearest)
plt.draw()
plt.pause(2) #pause to see initial state
colors.append('green')
cmap = ListedColormap(colors) #green color added to represent shortest path
for node in path:
    if obstacles[node[0], node[1]] == 1: #if obstacle detected continue
        continue
    obstacles[node[0], node[1]] = 2 #giving the shortest path nodes to value 2 for a different color
    plt.imshow(obstacles, cmap=cmap, interpolation='nearest') #used to represent path with new color
    plt.draw()
    plt.pause(0.1) #for smooth effect

time.sleep(3) #wait for 3 seconds before exiting