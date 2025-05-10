import copy
from collections import deque
import heapq
import itertools
import time

def computeManhattanDistance(state_grid, goal_positions):
    # Gets the total distance from each soldier and their goal position
    distance = 0
    for i in range(len(state_grid)):
        for j in range(len(state_grid[i])):
            value = state_grid[i][j]
            if value > 0:
                goal_i, goal_j = goal_positions[value]
                distance += abs(i - goal_i) + abs(j - goal_j)
    return distance

# Find moves is a function to help find valid soldiers to move
# It checks if a soldier is at a position and has empty spaces around it,
# and generates all possible sliding moves in each direction until an obstacle is hit.
def find_moves(state):
    # Valid soldiers contains a list of all valid soldiers to move each entry will be a list of x,y
    valid_soldiers = []

    for i in range(len(state)):
        for j in range(len(state[i])):
            # Move soldier as far as empty spaces exist
            if state[i][j] > 0:
                up = i - 1
                while up >= 0 and state[up][j] == 0:
                    up -= 1
                if up + 1 != i:
                    valid_soldiers.append([i, j, up + 1, j])

                down = i + 1
                while down < len(state) and state[down][j] == 0:
                    down += 1
                if down - 1 != i:
                    valid_soldiers.append([i, j, down - 1, j])

                left = j - 1
                while left >= 0 and state[i][left] == 0:
                    left -= 1
                if left + 1 != j:
                    valid_soldiers.append([i, j, i, left + 1])

                right = j + 1
                while right < len(state[i]) and state[i][right] == 0:
                    right += 1
                if right - 1 != j:
                    valid_soldiers.append([i, j, i, right - 1])

    return valid_soldiers

def generate_all_moves(state_value):
    # Returns all possible valid states
    valid_moves = find_moves(state_value)
    state_list = []
    for move in valid_moves:
        i = move[0]
        j = move[1]
        new_i = move[2]
        new_j = move[3]
        new_state = copy.deepcopy(state_value)
        new_state[new_i][new_j] = new_state[i][j]
        new_state[i][j] = 0
        state_list.append(new_state)
    return state_list

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

        if i < len(path) - 1:
            print("TO NEXT")
        if i == len(path) - 1:
            print("GOAL")

class State():
    
    def __init__(self, state_value, parent=None, depth=0):
        self.state_value = state_value
        self.depth = depth
        self.parent = parent

def Uniform_Cost_Search(initial_state, goal_state):
    # Expand the node with the lowest g(n) cost
    frontier = deque([initial_state])
    seen_states = set()
    # Initialize node counter
    node_expansions = 0

    while frontier:
        current_state = frontier.popleft()
        node_expansions += 1

        #print("Best state to expand to with g(n) = " + str(current_state.depth) + " is: ")

        for row in current_state.state_value:
            print(row)
        if current_state.state_value == goal_state:
            print("FOUND")
            print("Goal reached!")
            print("Solution depth:", current_state.depth)
            print("Path to solution: ")
            print_solution_path(current_state)
            print("Total nodes expanded:", node_expansions)
            return
        possible_moves = generate_all_moves(current_state.state_value)
        for next_state_value in possible_moves:
            state_tuple = []
            for row in next_state_value:
                state_tuple.append(tuple(row))
            state_tuple = tuple(state_tuple)
            if state_tuple not in seen_states:
                seen_states.add(state_tuple)
                next_state = State(next_state_value, current_state, current_state.depth + 1)
                frontier.append(next_state)
    print("Goal not reached.")
    print("Total nodes expanded:", node_expansions)

def AStar_Search(initial_state, goal_state):
    # Expands the node with the lowest cost (f(n) = g(n) + h(n))
    goal_positions = {}
    node_expansions = 0

    for i in range(len(goal_state.state_value)):
        for j in range(len(goal_state.state_value[i])):
            value = goal_state.state_value[i][j]
            if value > 0:
                goal_positions[value] = (i, j)

    frontier = []
    counter = itertools.count()
    seen_states = set()

    # Compute heuristic and f(n)
    initial_h = computeManhattanDistance(initial_state.state_value, goal_positions)
    # Push new state to frontier
    heapq.heappush(frontier, (initial_h, next(counter), initial_state))

    while frontier:
        current_cost, cost_counter, current_state = heapq.heappop(frontier)
        node_expansions += 1

        #print("Best state to expand to with f(n) = g(n) + h(n) = " + str(current_cost) + " is:")

        for row in current_state.state_value:
            continue
        if current_state.state_value == goal_state.state_value:
            print("FOUND")
            print("Goal reached!")
            print("Solution depth:", current_state.depth)
            print("Path to solution: ")
            print_solution_path(current_state)
            print("Total nodes expanded:", node_expansions)
            return
        next_moves = generate_all_moves(current_state.state_value)
        for next_state_value in next_moves:
            state_tuple = []
            for row in next_state_value:
                state_tuple.append(tuple(row))
            state_tuple = tuple(state_tuple)
            if state_tuple not in seen_states:
                seen_states.add(state_tuple)
                next_state = State(next_state_value, current_state, current_state.depth + 1)
                heuristic = computeManhattanDistance(next_state_value, goal_positions)
                total_cost = next_state.depth + heuristic
                heapq.heappush(frontier, (total_cost, next(counter), next_state))
    print("Goal not reached.")
    print("Total nodes expanded:", node_expansions)

def mainUI():
    goal_state = [[-1,-1,-1,0,-1,0,-1,0,-1,-1],
                    [1,2,3,4,5,6,7,8,9,0]]
    goal_state = State(goal_state)
    
    initial_state = []
    
    print("Welcome to mtrev008's Nine Men in a Trench puzzle solver!")
    choice = int(input('Type “1” to use a default puzzle, or “2” to enter your own puzzle: '))

    while choice != 1 and choice != 2:
        choice = input('Invalid input! Chose "1" to use a default puzzle, or "2" to enter your own puzzle: ')

    if choice == 2:
        while True:
            print("Great! Now enter 10 numbers (separated by a space) to represent the recesses. Enter 0 for the " \
            "empty recesses and -1 for the unavailable recesses.")
            recesses = input()
            recesses = recesses.strip().split()
            recesses = [int(x) for x in recesses]
            # print(recesses)

            if len(recesses) != 10:
                print("Invalid input! Enter only 10 numbers")
                continue

            print("Great! Now enter 10 numbers (separated by a space) to represent the trenches. Enter 0 for the " \
            "empty trench.")
            trenches = input()
            trenches = trenches.strip().split()

            trenches = [int(x) for x in trenches]
            #print(trenches)

            if len(trenches) != 10:
                print("Invalid input! Enter only 10 numbers")
                continue

            # Valid input
            break

        initial_state = [recesses, trenches]
        initial_state = State(initial_state)
    else:
        initial_state = [[-1,-1,-1,0,-1,0,-1,0,-1,-1],
                        [0,2,3,4,5,6,7,8,9,1]]
        initial_state = State(initial_state)


    print("Now choose your algorithm... ")
    print("1. Uniform Cost Search or 2. Astar With Manhattan Distance Heuristic")
    algorithm_choice = int(input())

    while algorithm_choice != 1 and algorithm_choice != 2:
        algorithm_choice = input('Invalid input! Chose "1" for Uniform Cost Search or "2" for Astar With Manhattan Distance Heuristic: ')


    if algorithm_choice == 1:
        print("Running Uniform Cost Search... ")
        start_time = time.time()
        Uniform_Cost_Search(initial_state, goal_state)
        print("Time elapsed:", round(time.time() - start_time, 2), "seconds")
    elif algorithm_choice == 2:
        print("Running Astar With Manhattan Distance Heuristic... ")
        start_time = time.time()
        AStar_Search(initial_state, goal_state)
        print("Time elapsed:", round(time.time() - start_time, 2), "seconds")


# Run main UI code
mainUI()