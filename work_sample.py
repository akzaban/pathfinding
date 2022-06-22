import numpy as np

#a basic node for A* Pathfinding 
class Node:

    def __init__(self, parent=None, position=None):
        
        #parent of the current node
        self.parent = parent
        
        #current position of the node in the grid
        self.position = position
        
        #cost from start to current node
        self.g = 0
        
        #heuristic based estimated cost for current node to end node
        self.h = 0
        
        #total cost of present node i.e. :  f = g + h
        self.f = 0
        
    def __eq__(self, other):
        return self.position == other.position

#This function returns the path of the search
def return_path(current_node,grid):
    
    path = []
    rows, columns = np.shape(grid)
    
    # here we create the initialized result grid with -1 in every position
    result = [[-1 for i in range(columns)] for j in range(rows)]
    current = current_node
    
    while current is not None:
        path.append(current.position)
        current = current.parent
        
    # Return reversed path since we need to show path from start to end 
    path = path[::-1]
    start_value = 0
    
    # increment the path from start to end found by A-star search in every step by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
        
    return result

#returns a list of tuples as a path from the given start to the given end in the given grid
def search(grid, cost, start, end):

    # Create start and end node with initized values for g, h and f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # nodes that are yet to be visited 
    yet_to_visit_list = []  
    # node that are already explored 
    visited_list = [] 
    
    # Add the start node
    yet_to_visit_list.append(start_node)
    
    # search movement is left-right-top-bottom 
    # 4 movements can be made from every positon

    move  =  [[-1, 0 ], # up
              [ 0, -1], # left
              [ 1, 0 ], # down
              [ 0, 1 ]] # right

    #finding shape of grid
    rows, columns = np.shape(grid)
    
    # condition to excute after reasonable number of steps and avoid infinite loop 
    outer_iterations = 0
    
    while len(yet_to_visit_list) > 0:
        
        # Every time a node is referred from yet_to_visit list, counter of limit operation is incremented
        outer_iterations += 1    

        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # add current node to visited list and pop from yet_to_visit_list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not
        if current_node == end_node:
            return return_path(current_node,grid)

        # Generate children from all adjacent squares
        children = []

        for new_position in move: 

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # check if within grid boundary
            if (node_position[0] > (rows - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (columns -1) or 
                node_position[1] < 0):
                continue

            # Make sure if terrain is walkable
            if grid[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            
            # search visited list
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            #calculate f, g, and h values
            child.g = current_node.g + cost
            ## Heuristic costs calculated using eucledian distance
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + 
                       ((child.position[1] - end_node.position[1]) ** 2)) 

            child.f = child.g + child.h

            # if child is already in the yet_to_visit list and g cost is lower
            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)

#checks whether path exists or not to give relevant result
def result(grid,start,end,cost_per_step):
    
    path_grid = search(grid,cost_per_step,start,end)
    
    if path_grid==None:
        print("Unable to reach delivery point")
    else:
        path=[]
        for i in range(0,10):
            for j in range(0,10):
                if path_grid[i][j]!=-1:
                    path.append([i,j])
                    cost=path_grid[i][j]
        print("Path is: ", path)
        print("Cost is: ", cost)
    

if __name__ == '__main__':
    
    start = [0, 0] # starting position
    end = [9,9] # ending position
    cost = 1 # cost per movement

    grid = [[0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 1, 0, 1, 0],
     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
     [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
     [1, 0, 0, 1, 1, 0, 0, 1, 1, 0],
     [0, 0, 0, 0, 1, 0, 0, 1, 1, 0]]
    
    
    #example where there is no solution
    grid2 = [[0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 1, 0, 1, 0],
     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
     [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
     [1, 0, 0, 1, 1, 0, 0, 1, 1, 1],
     [0, 0, 0, 0, 1, 0, 0, 1, 1, 0]]
    
    print("For Grid 1: ")
    result(grid,start,end,cost)
    print(" ")
    print("For Grid 2: ")
    result(grid2,start,end,cost)