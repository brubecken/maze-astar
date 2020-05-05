'''
Gabriel Alonzo Rusk
'''

import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index


    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
    
        draw(self.map, path[::-1], self.map_index)


    def goal_test(self, current_state):
        # your code goes here:
        if (current_state[0] == self.goal_state[0]) and (current_state[1] == self.goal_state[1]):
            return True
        else:
            return False


    def get_cost(self, current_state, next_state):
        # your code goes here:
        return 1


    def get_successors(self, state):
        # your code goes here:
        successors = []

        if state[0] > -0 and self.map[state[0] - 1, state[1]] == 1:
            temp = (state[0] - 1, state[1])
            successors.append(temp)

        if state[0] + 1 < self.m and self.map[state[0] + 1, state[1]] == 1:
            temp = (state[0] + 1, state[1])
            successors.append(temp)

        if state[1] > 0 and self.map[state[0], state[1] - 1] == 1:
            temp = (state[0], state[1] - 1)
            successors.append(temp)

        if state[1] + 1 < self.n and self.map[state[0], state[1] + 1] == 1:
            temp = (state[0], state[1] + 1)
            successors.append(temp)

        return successors


    # heuristics function
    def heuristics(self, state):
        # your code goes here:
        return abs(self.goal_state[0] - state[0]) + abs(self.goal_state[1] - state[1])


    # priority of node 
    def priority(self, node):
        # your code goes here:
        return node.cost_from_start + self.heuristics(node.state)

    
    # solve it
    def solve(self):
        # your code goes here:
        state = (self.start_state[0], self.start_state[1])
        node = Node(state, 0, None)
        self.visited.append(state)
        container = []
        count = 1 
        heappush(container, (count, count, node))
        while container:
            currentNode = heappop(container)[2]
            if self.goal_test(currentNode.state):
                self.draw(currentNode)
                break
            successors = self.get_successors(currentNode.state)
            for nextState in successors:
                visited = False
                for visitedStates in self.visited:
                    if visitedStates[0] == nextState[0] and visitedStates[1] == nextState[1]:
                        visited = True
                
                if not visited:
                    nextCost = currentNode.cost_from_start + self.get_cost(node.state, nextState)
                    nextNode = Node(nextState, nextCost, currentNode)
                    self.visited.append(nextState)
                    count += 1
                    heappush(container, (self.priority(nextNode), count, nextNode))

            
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1
    
    data = np.load('map_'+str(index)+'.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
    