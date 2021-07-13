"""my_4_supervisor_controller controller."""

from controller import Supervisor
#from controller import Robot
import struct
supervisor = Supervisor()
#robot = Robot()
timestep = 32
robot_node1 = supervisor.getFromDef("EPUCK1")
receiver1 = supervisor.getDevice("receiver1")
receiver1.enable(timestep)
receiver2 = supervisor.getDevice("receiver2")
receiver2.enable(timestep)
receiver3 = supervisor.getDevice("receiver3")
receiver3.enable(timestep)
receiver4 = supervisor.getDevice("receiver4")
receiver4.enable(timestep)
emitter = supervisor.getDevice("emitter")

robot_node = supervisor.getFromDef("EPUCK1")
t = supervisor.getTime()
trans_field = robot_node.getField("translation")
rot_field= robot_node.getField("rotation")

global_map_dict = {}
current_dir = []
next_dir = []
right = (0,1,0,-1.57071) 
up = (0,1,0,0)
left = (0,1,0,1.57071)
down = (0,1,0,3.14142)

node_path = [0, 1, 9, 8, 16, 17, 18, 19, 20, 12, 13, 14, 6, 5, 
4, 3, 2, 10, 11, 10, 2, 3, 4, 5, 6, 7, 15, 23, 22, 21, 29, 28, 
27, 26, 34, 35, 34, 26, 27, 28, 36, 37, 36, 28, 29, 21, 22, 30, 
38, 39, 31, 39, 38, 30, 22, 23, 15, 7, 6, 14, 13, 12, 20, 19, 18,
 17, 25, 24, 32, 33, 41, 40, 41, 42, 43, 44, 45, 46, 47, 46, 45, 
 53, 52, 60, 61, 62, 54, 55, 63, 55, 54, 62, 61, 60, 52, 53, 45,
  44, 43, 51, 50, 58, 59, 58, 50, 51, 43, 42, 41, 49, 48, 56, 57]

#Stack code from YouTube
class Stack():
    def __init__(self):
        self.stack = list()
    def push(self, item):
        self.stack.append(item)
    def pop(self):
        if len(self.stack) > 0:
            return self.stack.pop()
        else:
            return None
    def peek(self):
        if len(self.stack) > 0:
            return self.stack[len(self.stack)-1]
        else:
            return None
    def __str__(self):
        return str(self.stack)

stack = Stack()
#Graph code from GeeksForGeeks
# Python program for 
# validation of a graph
  
# import dictionary for graph
from collections import defaultdict
  
# function for adding edge to graph
graph = defaultdict(set)
def addEdge(graph,u,v):
    graph[u].add(v)
  
# definition of function
def generate_edges(graph):
    edges = []
    # for each node in graph
    for node in graph:         
        # for each neighbour node of a single node
        for neighbour in graph[node]:              
            # if edge exists then append
            edges.append((node, neighbour))
    return edges
    
def find_shortest_path(graph, start, end, path =[]):
        path = path + [start]
        if start == end:
            return path
        shortest = None
        for node in graph[start]:
            if node not in path:
                newpath = find_shortest_path(graph, node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest  
# declaration of graph as dictionary
#addEdge(graph,'a','c') 
# Driver Function call 
# to print generated graph
#print(generate_edges(graph))
visited = [False for i in range(63)]
path_used = []

#Functions
def next_node_generator(current_node):
    global visited
    global path_used
    global graph
    stack.push(current_node)
    visited[current_node] = True
    for node in graph[current_node]:
        if (not visited[node]):
            print("Node to be returned =",node)
            return node
    stack.pop()
    print("Back path to be returned =",stack.peek())
    return stack.pop()
    


def message_to_map_converion(X_pos,Z_pos,r_detail,u_detail,l_detail,d_detail):
    global global_map_dict
    surrounding = [r_detail,u_detail,l_detail,d_detail]
    global_map_dict[(X_pos,Z_pos)] = global_map_dict.get((X_pos,Z_pos),surrounding)

left_next_dir =   {right:list(up), up:list(left), left:list(down), down:list(right)}
right_next_dir =  {right:list(down), down:list(left), left:list(up), up:list(right)}
rotate_next_dir = {right:list(left), left:list(right), up:list(down), down:list(up)}

def graph_updation(current_node,r,u,l,d):
    #print("Current node",current_node)
    global graph
    if u and (current_node-8 not in graph[current_node]):
        addEdge(graph,current_node,current_node-8)
    if l and (current_node-1 not in graph[current_node]):
        addEdge(graph,current_node,current_node-1)
    if r and (current_node+1 not in graph[current_node]):
        addEdge(graph,current_node,current_node+1)
    if d and (current_node+8 not in graph[current_node]):
        addEdge(graph,current_node,current_node+8)

def which_side(current_node,next_node):
    if current_node+1 == next_node:
        print("Going right")
    elif current_node-8 == next_node:
        print("Going up")
    elif current_node-1 == next_node:
        print("Going left")
    elif current_node+8 == next_node:
        print("Going down")

def next_direction(current_node,next_node):
    global right
    global up
    global left
    global down
    if current_node+1 == next_node:
        return right
    elif current_node-8 == next_node:
        return up
    elif current_node-1 == next_node:
        return left
    elif current_node+8 == next_node:
        return down
    else:
        print("Invalid/Inbetween oriented value")

def conversion_between_node_and_position(length_of_arena,division=True):
    if division==False:
        length_of_arena*=2
    positions_numbers = []
    for i in range(length_of_arena-1,-1,-1):
        positions_numbers.append(-float(str(i)+".5"))
    for i in range(0,length_of_arena):
        positions_numbers.append(float(str(i)+".5"))
    node_to_position = {}
    mid_positions = []
    for z in positions_numbers:
        for x in positions_numbers:
            mid_positions.append((x,0,z))
    for i in range(0,64):
        node_to_position[i] = node_to_position.get(i,mid_positions[i])
    position_to_node = dict([(value, key) for key, value in node_to_position.items()])
    return node_to_position,position_to_node

#while supervisor.step(timestep) != -1:
length_of_arena = 4
node_to_position,position_to_node = conversion_between_node_and_position(4)
nodes = pow(length_of_arena,3)
    
positions = []
for node in node_path:
    positions.append(node_to_position[node])
    
timings = range(2,180,2)
current_dir = rot_field.getSFRotation()

next_node = 0
current_node = 0
for index,time in enumerate(timings):
        
    while supervisor.getTime() < time:
        if supervisor.step(timestep) == -1:
            quit()
    if receiver1.getQueueLength()>0:
        received_data = receiver1.getData()
        message = struct.unpack("? f f ? ? ? ?",received_data)
            #print("Message received")
        current_node = position_to_node[(message[1],0,message[2])]
        #print("Check =",position_to_node[(message[1],0,message[2])],(message[1],0,message[2]))
        graph_updation(position_to_node[(message[1],0,message[2])],message[3],message[4],message[5],message[6])
        message_to_map_converion(message[1],message[2],message[3],message[4],message[5],message[6])
        next_node = next_node_generator(current_node)
        #print("Message =",message)
        if message[0]:
            quit()
        #print("Map data =",global_map_dict)
        receiver1.nextPacket()
        #print("Queue lenght",receiver1.getQueueLength())
    if bool(graph):        
        #print("Graph",generate_edges(graph))
        print("Graph",graph)
        trans_field.setSFVec3f(list(node_to_position[next_node]))
        #print("Current node Robot is in =",next_node)
            
        (x,y,z,angle) = rot_field.getSFRotation()
        #print("Current orientation =",round(x),round(y),round(z),round(angle,4))
        #print("Current node =",current_node,"Next node =",node_path[index+1])
        rot_field.setSFRotation(list(next_direction(current_node,next_node)))
        robot_node.resetPhysics()
    #
    
    
    #if receiver2.getQueueLength()>0:
        #received_data = receiver1.getData()
        #message = struct.unpack("? f f ? ? ? ?",received_data)
        #message_to_map_converion(message[1],message[2],message[3],message[4],message[5],message[6])
        #print("Message =",message)
        #print("Map data =",global_map_dict)
        #receiver2.nextPacket()