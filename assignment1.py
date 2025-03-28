from time import time
import heapq
from debugpy.common.timestamp import current

from search import *
from assignment1aux import *


def read_initial_state_from_file(filename):
    # Task 1
    # Return an initial state constructed using a configuration in a file.

    # Open file and read info into "temp_list"
    map_file = open(f"{filename}")
    temp_list = []
    for line in map_file:
        temp_list.append(line.strip())
    map_file.close()

    # Create a list "map_list" to store the state of the garden.
    map_row = int(temp_list[0])
    map_col = int(temp_list[1])
    map_list = [['' for _ in range(map_col)] for _ in range(map_row)]

    # For every rock in garden, change the element from ""
    # to "rock" at the correct position in "map_list".
    for x in range(2, len(map_list)):
        temp_x = temp_list[x].split(",")
        i = int(temp_x[0])
        j = int(temp_x[1])
        map_list[i][j] = "rock"

    # Transform map to tuple format.
    map_tuple = tuple(tuple(every_row) for every_row in map_list)
    # Put map in format of tuple, initial position and direction together.
    state_init = (map_tuple, None, None)

    return state_init


class ZenPuzzleGarden(Problem):
    def __init__(self, initial):
        if type(initial) is str:
            super().__init__(read_initial_state_from_file(initial))
        else:
            super().__init__(initial)

    # Dictionary which create maps between string of directions and number
    dir2num = {0: "up", 1: "down", 2: "left", 3: "right"}
    num2dir = {"up": 0, "down": 1, "left": 2, "right": 3}

    # Dictionary which create maps between string of directions of actions and number
    dir2act_x = {"up": -1, "down": 1, "left": 0, "right": 0}
    dir2act_y = {"up": 0, "down": 0, "left": -1, "right": 1}

    def actions(self, state):
        # Task 2.1
        # Return a list of all allowed actions in a given state.

        action_list = []
        curr_map = state[0]
        curr_map = list(list(every_row) for every_row in curr_map)
        row = len(curr_map)
        col = len(curr_map[0])
        curr_pos = state[1]
        curr_dir = state[2]

        # If monk is in map
        if curr_pos:
            curr_x = curr_pos[0]
            curr_y = curr_pos[1]

            # Check 4 directions
            for d in range(4):
                # Monk can not turn 180°
                if d in [0, 1] and self.num2dir[curr_dir] in [0, 1]:
                    continue
                if d in [2, 3] and self.num2dir[curr_dir] in [2, 3]:
                    continue

                # Compute next position
                next_x = curr_x + self.dir2act_x[self.dir2num[d]]
                next_y = curr_y + self.dir2act_y[self.dir2num[d]]

                # If next position is in map, and it's unraked or not rock, append it
                if 0 <= next_x < row and 0 <= next_y < col and curr_map[next_x][next_y] == "":
                    action_list.append(((curr_x, curr_y), self.dir2num[d]))

                # If next position is go out of map, append it
                if (not 0 <= next_x < row) or not (0 <= next_y < col):
                    action_list.append(((curr_x, curr_y), self.dir2num[d]))
        else:
            # If monk is not in map, check all position in four edges
            for y in range(col):
                if curr_map[0][y] == "":
                    action_list.append(((0, y), "down"))
            for y in range(col):
                if curr_map[row - 1][y] == "":
                    action_list.append(((row - 1, y), "up"))
            for x in range(row):
                if curr_map[x][0] == "":
                    action_list.append(((x, 0), "right"))
            for x in range(row):
                if curr_map[x][col - 1] == "":
                    action_list.append(((x, col - 1), "left"))

        return action_list

    def result(self, state, action):
        # Task 2.2
        # Return a new state resulting from a given action being applied to a given state.

        # In this method, map could be changed, so use list format
        curr_map = state[0]
        curr_map = list(list(every_row) for every_row in curr_map)
        row = len(curr_map)
        col = len(curr_map[0])
        curr_pos = state[1]
        curr_x = action[0][0]
        curr_y = action[0][1]
        act_dir = action[1]
        while True:
            # Compute next position
            next_x = curr_x + self.dir2act_x[act_dir]
            next_y = curr_y + self.dir2act_y[act_dir]

            # If next position is in map and is unraked, move it
            if 0 <= next_x < row and 0 <= next_y < col and curr_map[next_x][next_y] == "":
                curr_map[curr_x][curr_y] = act_dir
                curr_x = next_x
                curr_y = next_y

            # If next position is out of map
            if next_x < 0 or next_x >= row or next_y < 0 or next_y >= col:
                curr_map[curr_x][curr_y] = act_dir
                curr_map = tuple(tuple(every_row) for every_row in curr_map)
                return tuple((curr_map, None, None))

            # If next position is rock or raked tile
            if curr_map[next_x][next_y] != "":
                curr_map = tuple(tuple(every_row) for every_row in curr_map)
                return tuple((curr_map, (curr_x, curr_y), act_dir))

    def goal_test(self, state):
        # Task 2.3
        # Return a boolean value indicating if a given state is solved.
        res_list = []
        curr_map = state[0]
        for row in curr_map:
            for ele in row:
                if ele == "":
                    res_list.append(False)
                else:
                    res_list.append(True)
        # If res_list has False, which means that there still unraked space in map, return 0
        return all(res_list)


# Task 3
# Implement an A* heuristic cost function and assign it to the variable below.
def astar_heuristic_cost(node):
    # Get map, it's width and height
    maP = node.state[0]
    row = len(maP)
    col = len(maP[0])
    ans1 = col
    ans2 = row

    # Check how many rows has the space which is unraked
    for x in range(row):
        allFinish = True
        for y in range(col):
            if maP[x][y] == "":
                allFinish = False
                break
        if allFinish:
            ans1 -= 1

    # Check how many columns has the space which is unraked
    for y in range(col):
        allFinish = True
        for x in range(row):
            if maP[x][y] == "":
                allFinish = False
                break
        if allFinish:
            ans2 -= 1

    # To ensure heuristic function is admissible, take the smallest possible value
    return min(ans1, ans2)


def beam_search(problem, f, beam_width):
    # Task 4
    # Implement a beam-width version A* search.
    # Return a search node containing a solved state.
    # Experiment with the beam width in the test code to find a solution.

    # Use heapq instead of priority queue
    f = memoize(f, 'f')
    init_node = Node(problem.initial)
    frontier = []

    # Initial node entry frontier
    heapq.heappush(frontier, (f(init_node), init_node))

    # record whether a state has been in checked
    explored = set()

    # Stop when frontier is empty
    while frontier:
        # Get current node from the front of heapq
        i, curr_node = heapq.heappop(frontier)

        # If the current node is in goal state, return it
        if problem.goal_test(curr_node.state):
            return curr_node

        # Mark current node to explored
        explored.add(curr_node.state)

        for child in curr_node.expand(problem):
            if child.state not in explored:
                # Check whether the child's state has been in frontier
                hasCheck = False
                for ele in frontier:
                    if ele[1] == child:
                        hasCheck = True
                        break

                # If not exist, add it to frontier
                if not hasCheck:
                    heapq.heappush(frontier, (f(child), child))
                else:
                    # If it exists, and it has lower f value, replace it
                    for ele in frontier:
                        if ele[1] == child and ele[0] < f(child):
                            frontier[i] = (f(child), child)
                            heapq.heapify(frontier)

        # If the length of frontier beyond beam width,
        # take the node with the number of beam width in the front
        if len(frontier) > beam_width:
            curr_frontier = []
            for i in range(beam_width):
                if not frontier:
                    break
                curr_frontier.append(heapq.heappop(frontier))
            frontier = curr_frontier

    # Return None if search failed
    return None


if __name__ == "__main__":
    # Task 1 test code

    print('The loaded initial state is visualised below.')
    visualise(read_initial_state_from_file('assignment1config.txt'))

    # Task 2 test code

    garden = ZenPuzzleGarden('assignment1config.txt')
    print('Running breadth-first graph search.')
    before_time = time()
    node = breadth_first_graph_search(garden)
    after_time = time()
    print(f'Breadth-first graph search took {after_time - before_time} seconds.')
    if node:
        print(f'Its solution with a cost of {node.path_cost} is animated below.')
        animate(node)
    else:
        print('No solution was found.')

    # Task 3 test code

    print('Running A* search.')
    before_time = time()
    node = astar_search(garden, astar_heuristic_cost)
    after_time = time()
    print(f'A* search took {after_time - before_time} seconds.')
    if node:
        print(f'Its solution with a cost of {node.path_cost} is animated below.')
        animate(node)
    else:
        print('No solution was found.')

    # Task 4 test code

    print('Running beam search.')
    before_time = time()
    node = beam_search(garden, lambda n: n.path_cost + astar_heuristic_cost(n), 50)
    after_time = time()
    print(f'Beam search took {after_time - before_time} seconds.')
    if node:
        print(f'Its solution with a cost of {node.path_cost} is animated below.')
        animate(node)
    else:
        print('No solution was found.')
