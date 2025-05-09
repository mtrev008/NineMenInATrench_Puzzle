import copy
from collections import deque


# Find moves is a function to help find valid soldiers to move
# It checks if a soldier is at a position and has empty spaces around it, so we can just check those moves 
# in the future
def find_moves(state):
    # Valid soldiers contains a list of all valid soldiers to move each entry will be a list of x,y
    valid_soldiers = [] 

    for i in range(len(state)):
        for j in range(len(state[i])):
            if(state[i][j] > 0):
                if(i > 0 and state[i-1][j]==0):
                    #print(f'found at {i,j}')
                    valid_soldiers.append([i,j])
                elif(i < len(state)-1 and state[i+1][j]==0):
                    #print(f'found at {i,j}')
                    valid_soldiers.append([i,j])
                elif(j > 0 and state[i][j-1]==0):
                    #print(f'found at {i,j}')
                    valid_soldiers.append([i,j])                    
                elif(j < len(state[i])-1 and state[i][j+1]==0):
                    #print(f'found at {i,j}')
                    valid_soldiers.append([i,j])     
                    
    return valid_soldiers

def print_solution_path(goal_state):
    path = []
    current_state = goal_state
    while current_state:
        path.append(current_state.state_value)
        current_state = current_state.parent

    path.reverse() 

    for i in range(len(path)):
        for row in path[i]:
            print(row)
            
        if (i < len(path)-1):
            print("TO NEXT")
        if (i == len(path)-1):
            print("GOAL")


class State():

    def __init__(self, state_value, parent=None, depth=0):
        self.state_value = state_value
        self.depth = depth
        self.parent = parent

    def moveUp(self):
        valid_moves = find_moves(self.state_value)
        state_list = []
        
        for moves in range(len(valid_moves)):
            i, j = valid_moves[moves]
            if(i > 0):
                if(self.state_value[i-1][j] == 0):
                    new_state = copy.deepcopy(self.state_value)
                    new_state[i-1][j] = new_state[i][j]
                    new_state[i][j] = 0
                    state_list.append(new_state)

        return state_list
    
    def moveDown(self):
        valid_moves = find_moves(self.state_value)
        state_list = []
        
        for moves in range(len(valid_moves)):
            i, j = valid_moves[moves]
            if(i < len(self.state_value)-1):
                if(self.state_value[i+1][j] == 0):
                    new_state = copy.deepcopy(self.state_value)
                    new_state[i+1][j] = new_state[i][j]
                    new_state[i][j] = 0
                    state_list.append(new_state)
        return state_list
    
    def moveRight(self):
        valid_moves = find_moves(self.state_value)
        state_list = []
        
        for moves in range(len(valid_moves)):
            i, j = valid_moves[moves]
            if(j < len(self.state_value[i])-1):
                if(self.state_value[i][j+1] == 0):
                    new_state = copy.deepcopy(self.state_value)
                    new_state[i][j+1] = new_state[i][j]
                    new_state[i][j] = 0
                    state_list.append(new_state)
        return state_list
    
    def moveLeft(self):
        valid_moves = find_moves(self.state_value)
        state_list = []
        
        for moves in range(len(valid_moves)):
            i, j = valid_moves[moves]
            if(j > 0):
                if(self.state_value[i][j-1] == 0):
                    new_state = copy.deepcopy(self.state_value)
                    new_state[i][j-1] = new_state[i][j]
                    new_state[i][j] = 0
                    state_list.append(new_state)
        return state_list


def Uniform_Cost_Search(initial_state, goal_state):
    frontier = deque([initial_state])
    seen_states = set()  # Using set instead of list like before for way faster lookup
    
    while frontier:
        current_state = frontier.popleft() 
        print("Best state to expand to with g(n) = " + str(current_state.depth) + " is: ")
        
        for i in current_state.state_value:
            print(i)
            
        if current_state.state_value == goal_state:
            print("FOUND")
            print("Goal reached!")
            print("Path to solution: ")
            print_solution_path(current_state)
            return

        possible_moves = []
        for state in current_state.moveUp():
            if state is not None:
                state_tuple = tuple(tuple(row) for row in state)
                if state_tuple not in seen_states:
                    seen_states.add(state_tuple)
                    temp_state = State(state, current_state, current_state.depth + 1)
                    frontier.append(temp_state)
                
        for state in current_state.moveDown():
            if state is not None:
                state_tuple = tuple(tuple(row) for row in state)
                if state_tuple not in seen_states:
                    seen_states.add(state_tuple)
                    temp_state = State(state, current_state, current_state.depth + 1)
                    frontier.append(temp_state)
                
        for state in current_state.moveLeft():
            if state is not None:
                state_tuple = tuple(tuple(row) for row in state)
                if state_tuple not in seen_states:
                    seen_states.add(state_tuple)
                    temp_state = State(state, current_state, current_state.depth + 1)
                    frontier.append(temp_state)
                
        for state in current_state.moveRight():
            if state is not None:
                state_tuple = tuple(tuple(row) for row in state)
                if state_tuple not in seen_states:
                    seen_states.add(state_tuple)
                    temp_state = State(state, current_state, current_state.depth + 1)
                    frontier.append(temp_state)
            
    print("Goal not reached.")


initial_state = [[-1,-1,-1,0,-1,0,-1,0,-1,-1],
                [0,2,3,4,5,6,7,8,9,1]]
goal_state = [[-1,-1,-1,0,-1,0,-1,0,-1,-1],
                [1,2,3,4,5,6,7,8,9,0]]


initial_state = State(initial_state)
goal_state = State(goal_state)


Uniform_Cost_Search(initial_state, goal_state)