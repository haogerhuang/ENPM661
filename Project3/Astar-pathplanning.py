import numpy as np
import cv2
import heapq
from tqdm import tqdm
import argparse

THETAS = [-60,-30,0,30,60]

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
            
class Node:
    def __init__(self, x, y, theta, g, h):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.h = h
        self.f = g+h
    def __lt__(self, other):
        return self.f < other.f
    def __gt__(self, other):
        return self.f > other.f
    def __eq__(self, other):
        return self.f == other.f
        

# Main class to perform A* search on the defined map
class Astar:
    def __init__(self, m, n, clearance, radius, thres, step_size=1, resolution=(0.5, 30)):
        self.clearance = clearance
        self.radius = radius
        self.thres = thres
        self.step_size = step_size
        self.resolution = resolution
        self.obstacle_list = []
        self.createObstacleSpace()
        self.Nodes = []
        self.m = m
        self.n = n
        self.visited = np.zeros([int((m+1)/resolution[0]),
                                 int((n+1)/resolution[0]),
                                 int(360/resolution[1])])
        self.map = np.zeros([self.visited.shape[1],self.visited.shape[0],3])+0.1
        self.path = []
        self.drawObstacles()
    def set_start_goal(self, start, goal):
        s_x,s_y,s_t = start
        g_x,g_y,g_t = goal

        self.start = start
        self.goal = goal

        # Check if start and goal positions are valid
        if not self.valid_loc(s_x,s_y,s_t) or not self.valid_loc(g_x,g_y,g_t): 
            print('Invalid input, drawing...')
            self.drawValidArea()
            self.draw_start_goal()
            
            print('Make sure the start and goal positions are in the green area.')
            cv2.imshow('Invalid input', self.map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            raise Exception("Invalid start position and goal position")

    # Draw start and goal positions
    def draw_start_goal(self):
        r = self.resolution[0]
        l = 10
        n = self.map.shape[0]
        x1 = int(self.start[0]/r) 
        y1 = int(n-self.start[1]/r)
        gx1 = int(self.goal[0]/r)
        gy1 = int(n-self.goal[1]/r)
         
        x2 = int(x1 + l*np.cos(np.pi/180*self.start[2]))
        y2 = int(y1 - l*np.sin(np.pi/180*self.start[2]))

        gx2 = int(gx1 + l*np.cos(np.pi/180*self.goal[2]))
        gy2 = int(gy1 - l*np.sin(np.pi/180*self.goal[2]))
        cv2.circle(self.map, (x1,y1), 3, (0,0,255), 2)
        cv2.circle(self.map, (gx1,gy1), 3, (0,0,255), 2)
        cv2.line(self.map, (x1,y1), (x2,y2), (0,0,255), 2)
        cv2.line(self.map, (gx1,gy1), (gx2,gy2), (0,0,255), 2)

    def drawObstacles(self):
        y = self.map.shape[0]
        r = self.resolution[0]
        clr = (255,0,0)
        cv2.line(self.map, (int(115/r),int(y-210/r)),(int(36/r),int(y-185/r)),clr,2)
        cv2.line(self.map, (int(115/r),int(y-210/r)),(int(80/r),int(y-180/r)),clr,2)
        cv2.line(self.map, (int(105/r),int(y-100/r)),(int(80/r),int(y-180/r)),clr,2)
        cv2.line(self.map, (int(105/r),int(y-100/r)),(int(36/r),int(y-185/r)),clr,2)

        cv2.line(self.map,(int(200/r),int(y-(100+70/3**0.5)/r)),(int(235/r),int(y-(100+35/3**0.5)/r)),(255,0,0),2)
        cv2.line(self.map,(int(235/r),int(y-(100-35/3**0.5)/r)),(int(235/r),int(y-(100+35/3**0.5)/r)),(255,0,0),2)
        cv2.line(self.map,(int(235/r),int(y-(100-35/3**0.5)/r)),(int(200/r),int(y-(100-70/3**0.5)/r)),(255,0,0),2)
        cv2.line(self.map,(int(165/r),int(y-(100-35/3**0.5)/r)),(int(200/r),int(y-(100-70/3**0.5)/r)),(255,0,0),2)
        cv2.line(self.map,(int(165/r),int(y-(100-35/3**0.5)/r)),(int(165/r),int(y-(100+35/3**0.5)/r)),(255,0,0),2)
        cv2.line(self.map,(int(200/r),int(y-(100+70/3**0.5)/r)),(int(165/r),int(y-(100+35/3**0.5)/r)),(255,0,0),2)

        cv2.circle(self.map, (int(300/r),int(y-185/r)), int(40/r), (255,0,0), 2)
    
    # Use when input is invalid
    def drawValidArea(self):
        m,n,_ = self.map.shape
        for i in range(m):
            for j in range(n):
                if self.valid_loc(j/2,i/2,0): self.map[m-i,j] = np.array([0,150,0])
                

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

    # Calculate heuristic based on (DISTANCE TO GOAL)/(STEP SIZE)
    def heuristic(self,x,y,theta):
        theta_r = self.resolution[1]
        theta_d = np.abs(self.goal[2]-theta)
        r_d = ((self.goal[0]-x)**2 + (self.goal[1]-y)**2)**0.5 - self.thres
        return np.abs(r_d)/self.step_size
        
    # Construct next position lists 
    # Next positions must be valid to be put in the list
    def nextNodes(self, node):
        nextList = []
        for theta in THETAS:
            next_theta = (node.theta + theta)%360
            next_x = np.cos(np.deg2rad(next_theta))*self.step_size + node.x
            next_y = np.sin(np.deg2rad(next_theta))*self.step_size + node.y
            if not self.valid_loc(next_x,next_y,next_theta): continue
            next_g = node.g + 1

            next_h = self.heuristic(next_x,next_y,next_theta)
            nextNode = Node(next_x,next_y,next_theta,next_g,next_h)
            nextList.append(nextNode)

        return nextList
   
    # Check if the location is a valid location: Not in obstacle or clearance area or visited
    def valid_loc(self,x,y,theta):
        total_clearance = self.clearance+self.radius
        if x < total_clearance or\
           x >= self.m-total_clearance or\
           y < total_clearance or\
           y >= self.n-total_clearance: return False

        grid_x = int(x) + int((np.ceil(x) - x) < 0.5)
        grid_y = int(y) + int((np.ceil(y) - y) < 0.5)

        grid_x = int(grid_x/self.resolution[0])
        grid_y = int(grid_y/self.resolution[0])
        grid_t = int(theta/self.resolution[1])
        
        if self.visited[grid_x,grid_y,grid_t] == 1: return False

        for obstacle in self.obstacle_list:
            inObstacle = True
            inObstacle = inObstacle and obstacle.inside((x,y),total_clearance)
            if inObstacle:
                return False
        
        return True

    def markVisited(self, node):
        x = node.x
        y = node.y
        theta = node.theta

        grid_x = int(x) + int((np.ceil(x) - x) < 0.5)
        grid_y = int(y) + int((np.ceil(y) - y) < 0.5)

        grid_x = int(grid_x/self.resolution[0])
        grid_y = int(grid_y/self.resolution[0])
        grid_t = int(theta/self.resolution[1])
        self.visited[grid_x,grid_y,grid_t] = 1
    
    # Check if current position is close to the goal within a certain threshold
    def reachGoal(self, node):
        d = (self.goal[0]-node.x)**2+(self.goal[1]-node.y)**2
        theta_d = np.abs(node.theta - self.goal[2])
        return d <= self.thres**2 and theta_d < self.resolution[1]
        
    
    # Main search algorithm
    def search(self):
        x,y,theta = self.start
        startNode = Node(x,y,theta,0,self.heuristic(x,y,theta))
        # Mark start node as visited
        self.Nodes.append((startNode, -1))
        self.markVisited(startNode)
        # Initialize Priority Queue with the start node
        pq = [(startNode, 0)]
        reach = False
        while(len(pq) > 0 and not reach):
            # Pop the node with the smallest cost
            curNode, idx = heapq.heappop(pq)
            # Get next valid positions
            nextNodes = self.nextNodes(curNode)
            for node in nextNodes:
                # Mark next nodes as visited
                self.markVisited(node)
                self.Nodes.append((node, idx))
                # Push next nodes to priority queue 
                heapq.heappush(pq, (node, len(self.Nodes)-1))

                # Terminate if reach goal
                if self.reachGoal(node): 
                    reach = True
                    idx = len(self.Nodes)-1
                    break
        if not reach:
            self.draw_start_goal()
            
            cv2.imshow('No path found', self.map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            raise Exception("No path found")
                        
        # Backtrack to find the path
        self.generate_path(idx)
        self.path = self.path[::-1]

    # Recursive function that finds the path from goal to start
    def generate_path(self, cur_idx):
        if cur_idx == -1: return

        node, parent_idx = self.Nodes[cur_idx]
        self.path.append(node)
        self.generate_path(parent_idx)
            
    # Store search process to video
    def visualize(self, file_name):
        y,x,_ = self.map.shape
        r = self.resolution[0]
        self.draw_start_goal()

        cap = cv2.VideoCapture(0)
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        out = cv2.VideoWriter(file_name, fourcc, 40.0, (x,y))
        out.write(self.map.astype(np.uint8))

        for i, (node, idx) in enumerate(tqdm(self.Nodes)):
            if idx == -1: continue
            parent_node, _ = self.Nodes[idx]
            cv2.line(self.map, (int(node.x/r), int(y-node.y/r)),\
                        (int(parent_node.x/r), int(y-parent_node.y/r)),(255,255,255),1)
            if i % 50 == 0:
                out.write(self.map.astype(np.uint8))

        self.draw_start_goal()

        prev = None
        for i, node in enumerate(tqdm(self.path)):
            if prev is not None:
                cv2.line(self.map, prev, (int(node.x/r),int(y-node.y/r)), (0,0,255),2)
            prev = (int(node.x/r),int(y-node.y/r))
            out.write(self.map.astype(np.uint8))


        out.release()
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--start', nargs='+', type=int, required=True)
    parser.add_argument('-g','--goal', nargs='+', type=int, required=True)
    parser.add_argument('-c','--clearance', type=int, default=5)
    parser.add_argument('-r', '--radius', type=int, default=10)
    parser.add_argument('-step', '--step', type=int, default=1)
    parser.add_argument('-f','--file_name', type=str, default="visualize")
    args = parser.parse_args()

    if len(args.start) != 3:
        raise Exception("{} is an invalid start position. Should be size of 3".format(args.start))
    if len(args.goal) != 3:
        raise Exception("{} is an invalid goal position. Should be size of 3".format(args.goal))

    # Initialize Map
    astar = Astar(400, 250, args.clearance, args.radius, 1.5, args.step)
    # Specify the start position and end position
    astar.set_start_goal(tuple(args.start), tuple(args.goal))

    astar.search()
    
    # Save to video
    astar.visualize("{}.mp4".format(args.file_name))

