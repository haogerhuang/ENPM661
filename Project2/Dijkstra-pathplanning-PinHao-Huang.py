import numpy as np
import cv2
import heapq
from tqdm import tqdm
import argparse

action_set = [(1,0,1),(0,1,1),(-1,0,1),(0,-1,1),
                  (1,1,1.4),(1,-1,1.4),(-1,1,1.4),(-1,-1,1.4)]


# A class that describes a line
class Line:
    def __init__(self):
        self.vertical = False  # True if it's a vertical line
        self.pnt1 = None
        self.pnt2 = None
        self.a = None
        self.b = -1
        self.c = None

    # Construct line from two points and convert to: ax + by + c = 0
    def from2pnts(self, pnt1, pnt2):
        self.pnt1 = pnt1
        self.pnt2 = pnt2
        self.x = 0
        # Line is vertical
        if (self.pnt2[0] - self.pnt1[0]) == 0: 
            self.a = float('inf')
            self.vertical = True
            self.x = pnt1[0] # Eq: x = {}

        # Line is not vertical
        else: self.a = (self.pnt2[1]-self.pnt1[1])/(self.pnt2[0]-self.pnt1[0])
            
        self.c = pnt1[1]-self.a*pnt1[0]

    # Construct line from slope and one point and convert to: ax + by + c = 0
    def from1pnt(self, m, pnt):
        self.pnt1 = pnt
        self.a = m
        self.c = pnt[1]-self.a*pnt[0]

    def slope(self):
        if (self.pnt2[0] - self.pnt1[0]) == 0: return float('inf')
        return (self.pnt2[1]-self.pnt1[1])/(self.pnt2[0]-self.pnt1[0])

    # Shortest distance between th given point to the line
    def distance(self, pnt):
        if self.vertical: return abs(pnt[0] - self.x)
        x,y = pnt
        return abs(self.a*x+self.b*y+self.c)/(self.a**2+self.b**2)**(0.5)
        

    # True if the point is above the line
    def larger(self, pnt):
        x,y = pnt
        if self.vertical: return x > self.x
        return (y-self.pnt1[1]) > self.a*(x-self.pnt1[0])

    # True if the point is below the line
    def smaller(self, pnt):
        x,y = pnt
        if self.vertical: return x < self.x
        return (y-self.pnt1[1]) < self.a*(x-self.pnt1[0])
        
# A class that describes a circle
class Circle:
    def __init__(self, cen, r):
        self.cen = cen
        self.r = r
    # True if the pnt is in the circle (with clearance)
    def inside(self, pnt, clearance=0):
        return self.distance(pnt) < clearance

    # Distance between given point to the origin of the circle
    def distance(self, pnt):
        x,y = pnt
        return ((x-self.cen[0])**2 + (y-self.cen[1])**2)**0.5 - self.r
        

# A class that describes a polygon
class Polygon_Obstacle:
    def __init__(self):
        # List contains Line objects that enclose the polygon
        self.lines = []
        # List contains information to enclose the clearance area
        self.clearance = []
    def construct(self, l):
        """
         Construct the polygon given list:
         The list looks like: [(pnt1,pnt2,larger),...]
         pnt1,pnt2 are the points that define a line 
         larger={True/False} determine where the point should be (above/below) 
         to be considered inside the polygon
        """
        for pnt1,pnt2,larger in l:
            # Define line
            line = Line()
            line.from2pnts(pnt1,pnt2)
            self.lines.append([line, larger])

            # Define clearance area
            c_lines = []
            c_lines.append([line])

            # line1 and line2 are orthogonal to the line defined above
            line1 = Line()
            line2 = Line()
            line1.from1pnt(-1/line.a, pnt1)
            line2.from1pnt(-1/line.a, pnt2)
            if pnt1[1] > pnt2[1]:
                c_lines.append([line1, False])
                c_lines.append([line2, True])
            else:
                c_lines.append([line1, True])
                c_lines.append([line2, False])
            
            self.clearance.append(c_lines)

    # Determine whether a point is inside the polygon
    def inside(self, pnt, clearance=0):

        isInside = True
        for line,larger in self.lines:
            # The point should be above the line and is above the line
            if larger: isInside = isInside and line.larger(pnt) 
            # The point should be below the line and is below the line
            else: isInside = isInside and line.smaller(pnt)

        # If the point is inside the polygon, return 
        if isInside: return True

        # The point is not inside the polygon, check if the point is 
        # in the clearance area
        for c_lines in self.clearance:
            line = c_lines[0][0]
            # Check if the point is close enough to the line
            inThis = (line.distance(pnt) < clearance) 
            # If not, ignore
            if not inThis: continue

            # Circle clearance areas with the vertexs as origin and clearance as radius
            circle1 = Circle(line.pnt1, clearance)
            circle2 = Circle(line.pnt2, clearance)

            # Check if the point falls into the clearance area
            for line in c_lines[1:]:
                if line[1]: inThis = inThis and line[0].larger(pnt)
                else: inThis = inThis and line[0].smaller(pnt)
            if inThis or circle1.inside(pnt) or circle2.inside(pnt): return True


        return False
            

# Main class to perform dijkstra search on the defined map
class dijkstra:
    def __init__(self, m, n, clearance):
        self.clearance = clearance
        self.obstacle_list = []
        self.createObstacleSpace()
        self.Nodes = []
        self.visited = np.zeros([m+1,n+1])
        self.map = np.zeros([m+1,n+1,3])+50
        self.drawMap()
        self.path = []
    
    # Visualize map and obstacles
    def drawMap(self):    
        m,n,_ = self.map.shape
        for i in range(m):
            for j in range(n):
                if not self.valid_loc((i,j), True): 
                    self.map[i,j,1] = 255
        

    # Construct Obstacle Spaces
    def createObstacleSpace(self):
        P1 = Polygon_Obstacle()
        P1.construct([((36,185),(115,210),False),
                        ((80,180),(115,210),True),
                        ((36,185),(80,180),True)])
        self.obstacle_list.append(P1)

        P2 = Polygon_Obstacle()
        P2.construct([((36,185),(105,100),True),
                        ((80,180),(105,100),False),
                        ((36,185),(80,180),False)])
        self.obstacle_list.append(P2)

        P3 = Polygon_Obstacle()
        P3.construct([((235,100+35/3**0.5), (235,100-35/3**0.5),False),
                       ((165,100+35/3**0.5), (165,100-35/3**0.5),True),
                        ((165,100+35/3**0.5),(200,100+70/3**0.5),False),
                        ((235,100+35/3**0.5),(200,100+70/3**0.5),False),
                        ((165,100-35/3**0.5),(200,100-70/3**0.5),True),
                        ((235,100-35/3**0.5),(200,100-70/3**0.5),True)])
        self.obstacle_list.append(P3)
        self.obstacle_list.append(Circle((300,185),40))

        
    # Construct next position lists 
    # Next positions must be valid to be put in the list
    def nextPos(self, pos):
        l = []
        for o_x,o_y,cost in action_set:
            n_x = pos[0] + o_x
            n_y = pos[1] + o_y
            if self.valid_loc((n_x,n_y)): 
                l.append((n_x,n_y,cost))
        return l
   
    # Check if the location is a valid location: Not in obstacle or clearance area or visited
    # If in visualization mode, ignore clearance
    def valid_loc(self, loc, visual=False):
        if not visual:
            if loc[0] < self.clearance or loc[0] >= self.visited.shape[0]-self.clearance or loc[1] < self.clearance or loc[1] >= self.visited.shape[1]-self.clearance: return False

        if self.visited[loc[0],loc[1]] == 1: return False

        for obstacle in self.obstacle_list:
            inObstacle = True
            if visual:
                inObstacle = inObstacle and obstacle.inside(loc,0)
            else:
                inObstacle = inObstacle and obstacle.inside(loc,self.clearance)
            if inObstacle:
                return False
        
        return True
    
    # Main search algorithm
    def search(self, start, goal):
        if not self.valid_loc(start) or not self.valid_loc(goal): 
            raise Exception("Invalid start position and goal position")

        
        # Mark start node as visited
        self.Nodes.append((start, -1))
        self.visited[start[0],start[1]] = 1
        # Initialize Priority Queue with the start node
        pq = [(0, 0)]
        reachGoal = False
        while(len(pq) > 0 and not reachGoal):
            # Pop the node with the smallest cost
            cost_sf, idx = heapq.heappop(pq)
            pos = self.Nodes[idx][0]
            # Get next valid positions
            nextNodes = self.nextPos(pos)
            for n_x,n_y,cost in nextNodes:
                # Mark next nodes as visited
                self.visited[n_x,n_y] = 1
                self.Nodes.append(((n_x,n_y), idx))
                # Push next nodes to priority queue 
                heapq.heappush(pq, (cost_sf+cost, len(self.Nodes)-1))

                # Terminate if reach goal
                if n_x == goal[0] and n_y == goal[1]: 
                    reachGoal = True
                    idx = len(self.Nodes)-1
                    break
                        
        # Backtrack to find the path
        self.generate_path(idx)
        self.path = self.path[::-1]

    # Recursive function that finds the path from goal to start
    def generate_path(self, cur_idx):
        if cur_idx == -1: return

        loc, parent_idx = self.Nodes[cur_idx]
        self.path.append(loc)
        self.generate_path(parent_idx)
            
        
    # Visualize the algorithm and store to a video
    def visualize(self, file_name):
        m,n,_ = self.map.shape
        cap = cv2.VideoCapture(0)
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        out = cv2.VideoWriter(file_name, fourcc, 40.0, (m,n))
        cur_frame = np.transpose(self.map, (1,0,2))[::-1].astype(np.uint8)
        out.write(cur_frame)
        for i, (loc,_) in enumerate(tqdm(self.Nodes)):
            self.map[loc[0],loc[1],:] = 255 
            if i % 50 == 0:
                cur_frame = np.transpose(self.map, (1,0,2))[::-1].astype(np.uint8)
                out.write(cur_frame)

        for i, loc in enumerate(tqdm(self.path)):
            self.map[loc[0],loc[1],0] = 0
            self.map[loc[0],loc[1],1] = 0
            self.map[loc[0],loc[1],2] = 255
            if i % 5 == 0:
                cur_frame = np.transpose(self.map, (1,0,2))[::-1].astype(np.uint8)
                out.write(cur_frame)
        out.release()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--start', nargs='+', type=int, required=True)
    parser.add_argument('-g','--goal', nargs='+', type=int, required=True)
    parser.add_argument('-c','--clearance', type=int, default=5)
    parser.add_argument('-f','--file_name', type=str, default="visualize")
    args = parser.parse_args()

    if len(args.start) != 2:
        raise Exception("{} is an invalid start position. Should be size of 2".format(args.start))
    if len(args.goal) != 2:
        raise Exception("{} is an invalid goal position. Should be size of 2".format(args.goal))

    # Initialize Map
    d = dijkstra(400,250,args.clearance)
    # Specify the start position and end position

    
    d.search(tuple(args.start),tuple(args.goal))
    
    # Save to video
    d.visualize("{}.mp4".format(args.file_name))

