import numpy as np
import heapq
import argparse

def state2str(state):
    return str(state.T.reshape(-1))[1:-1]

# Get the blank space location
def getLocation(state):
    # Flatten the matrix and get the index of the smallest value
    idx = np.argmin(state.reshape(-1))
    # Transform the 1-d indice into 2-d indices
    return (int(idx/3), idx%3)

def readFile(filename):
    data = np.loadtxt(filename)
    if data.shape != (2,9):
        raise Exception("Invalid Input format. {:s} should contains only two lines. i.e. \n1 4 7 5 0 8 2 3 6\n1 4 7 2 5 8 3 6 0".format(filename))
    return data[0].astype(int).reshape(3,3).T, data[1].astype(int).reshape(3,3).T

# Get the manhattan distance between two states
def distance(state1, state2):

    # Flatten the matrixs
    state1 = state1.reshape(-1)
    state2 = state2.reshape(-1)
    # Get the indexs that sorted the vectors
    sort_idx1 = state1.argsort()
    sort_idx2 = state2.argsort()

    # Get the horizontal and vertical distance for each value
    distance = np.abs((sort_idx1/3).astype(int) - (sort_idx2/3).astype(int)) +\
                np.abs(sort_idx1%3 - sort_idx2%3)
    return distance.sum()
    

# Node for breadth first search
class Node:
    def __init__(self, state, idx, parent_idx):
        self.state = state
        self.idx = idx
        self.parent_idx = parent_idx

# Node for A star search
class A_Node(Node):
    def __init__(self, state, idx, parent_idx, g, h):
        super().__init__(state, idx, parent_idx)
        self.g = g # Cost to come
        self.h = h # Heuristic (Estimate distance to goal)
        self.f = g+h
    def __lt__(self, other):
        return self.f < other.f
    def __gt__(self, other):
        return self.f > other.f

class Puzzle:
    def __init__(self):
        # Dictionary that stores the visited states using string 
        # key: string representation of state, value: node index in self.store 
        self.visited = {}
        # List that stores visited nodes
        self.store = []

        self.action_list = [(0,1),(1,0),(0,-1),(-1,0)]

    def search(self, initial_state, goal_state, mode):
        if mode == 'a*':
            self.astar_search(initial_state, goal_state)
        else:
            self.bfs_search(initial_state, goal_state)

    def bfs_search(self, initial_state, goal_state):
        # Start node
        init_node = Node(initial_state, 0, 0)
        # Store start node
        self.visited[state2str(initial_state)] = 0
        self.store.append(init_node)
        # Initialize queue
        Q = [init_node]
        reachGoal = False
        str_goal_state = state2str(goal_state)

        while(len(Q)>0 and not reachGoal):
            # Pop node from front
            cur_node = Q.pop(0)
            # Get possible next states given current state
            nextStates = self.PossibleActions(cur_node)
            for state in nextStates:
                node = Node(state, len(self.store), cur_node.idx)
                str_state = state2str(state)
                # If reach goal
                if str_state == str_goal_state:
                    cur_node = node
                    reachGoal = True
                    break
                    
                # If node is visited before, ignore.
                if str_state in self.visited: continue
                # Mark current node as visited
                self.visited[str_state] = node.idx
                self.store.append(node)
                # Else, append to queue
                Q.append(node)
            
        # Backtrack to generate path and write to file
        self.generate_path(cur_node)
        # Write visited nodes info to file
        self.writeNodes()
    
    def astar_search(self, initial_state, goal_state):
        init_node = A_Node(initial_state, 0, 0, 0, distance(initial_state, goal_state))
        # Store start node
        self.visited[state2str(initial_state)] = 0
        self.store.append(init_node)
        # Push start node to priority queue
        pq = [init_node]
        reachGoal = False
        str_goal_state = state2str(goal_state)
        while(len(pq) > 0 and not reachGoal):
            # Pop node with smallest f from priority queue
            cur_node = heapq.heappop(pq)
            # Get possible next states from current state
            nextStates = self.PossibleActions(cur_node)
            for state in nextStates:
                str_state = state2str(state)
                # Estimate distance between this state and the goal state
                h = distance(state, goal_state)
                node = A_Node(state, len(self.visited), cur_node.idx, cur_node.g+1, h)
                # Reach goal
                if str_state == str_goal_state:
                    cur_node = node     
                    reachGoal = True
                    break

                # If the next state has been visited before
                if str_state in self.visited:    
                    idx = self.visited[str_state]
                    old_node = self.store[idx]
                    # Update the node if the new node has smaller f and push to pq
                    if node.f < old_node.f: 
                        node.idx = old_node.idx
                        heapq.heappush(pq, node)
                        self.store[idx] = node
                # Else, mark this node as visited and push to pq
                else:
                    self.visited[str_state] = node.idx
                    self.store.append(node)
                    heapq.heappush(pq, node)
            
        # Backtrack to generate path and write to file
        self.generate_path(cur_node)
        # Write visited nodes info to file
        self.writeNodes()


    # Function that backtracks to the start node
    def generate_path(self, cur_node):
        path = []
        # Backtrack until reach node with index 1
        while(cur_node.idx != 0):
            path.append(cur_node) 
            cur_node = self.store[cur_node.parent_idx]
        # Do not forget to push the node with index 1
        path.append(cur_node)
        # Write path to file
        f = open('nodePath.txt', 'w')
        for node in path[::-1]:
            f.write(state2str(node.state)+'\n')

    # Write visited nodes to file
    def writeNodes(self):
        f = open('NodesInfo.txt', 'w')
        f.write('Node_index Parent_Node_index Cost\n')
        f1 = open('Nodes.txt', 'w')
        line = '{} {} 0'
        for node in self.store:
            f.write(line.format(node.idx+1, node.parent_idx+1)+'\n')
            f1.write(state2str(node.state)+str('\n'))
        
    # Return possible next states given current node
    def PossibleActions(self, currentNode):
        cur_i,cur_j = getLocation(currentNode.state)
        nextStates = []
        for i,j in self.action_list:
            # Choose nearby indices to be swapped
            next_i = cur_i+i
            next_j = cur_j+j
            # Check if swap indices are valid
            if next_i >= 0 and next_j >= 0 and next_i < 3 and next_j < 3:
                next_state = currentNode.state.copy()
                next_state[cur_i][cur_j] = next_state[next_i][next_j]
                next_state[next_i][next_j] = 0
                nextStates.append(next_state)
                      
        return nextStates


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--input', required=True, type=str, help='input text file')
    parser.add_argument('-m', '--mode', default='a*', choices=['a*','bfs'], help='Choose search algorithm')
    args = parser.parse_args()

    initial, goal = readFile(args.input)
    
    p = Puzzle()
    p.search(initial, goal, args.mode)

