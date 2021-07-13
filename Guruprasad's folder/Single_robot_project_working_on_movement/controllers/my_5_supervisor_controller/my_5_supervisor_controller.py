"""my_5_supervisor_controller controller."""
from controller import Supervisor

import struct
supervisor = Supervisor()

timestep = 32
robot_node1 = supervisor.getFromDef("EPUCK1")
robot_node2 = supervisor.getFromDef("EPUCK2")
robot_node3 = supervisor.getFromDef("EPUCK3")
robot_node4 = supervisor.getFromDef("EPUCK4")
receiver1 = supervisor.getDevice("receiver1")
receiver1.enable(timestep)
receiver2 = supervisor.getDevice("receiver2")
receiver2.enable(timestep)
receiver3 = supervisor.getDevice("receiver3")
receiver3.enable(timestep)
receiver4 = supervisor.getDevice("receiver4")
receiver4.enable(timestep)
emitter = supervisor.getDevice("emitter")

trans_field1 = robot_node1.getField("translation")
rot_field1= robot_node1.getField("rotation")
trans_field2 = robot_node2.getField("translation")
rot_field2= robot_node2.getField("rotation")
trans_field3 = robot_node3.getField("translation")
rot_field3= robot_node3.getField("rotation")
trans_field4 = robot_node4.getField("translation")
rot_field4= robot_node4.getField("rotation")

global_map_dict = {}
right = (0,1,0,-1.57071) 
up = (0,1,0,0)
left = (0,1,0,1.57071)
down = (0,1,0,3.14142)

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

stack1 = Stack()
stack2 = Stack()
stack3 = Stack()
stack4 = Stack()

stack_access_dict = {1:stack1,2:stack2,3:stack3,4:stack4}

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
visited = [False for i in range(1000)]


#Functions
def next_node_generator(current_node,robot_number):
    global visited
    global graph
    global stack_access_dict
    stack_access_dict[robot_number].push(current_node)
    visited[current_node] = True
    for node in graph[current_node]:
        if (not visited[node]):
            #print("Node to be returned =",node)
            return node
    stack_access_dict[robot_number].pop()
    #print("Back path to be returned =",stack.peek())
    return stack_access_dict[robot_number].pop()
    


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
    
timings = range(2,180,2)

next_node1 = 0
current_node1 = 0
next_node2 = 0
current_node2 = 0
next_node3 = 0
current_node3 = 0
next_node4 = 0
current_node4 = 0
for index,time in enumerate(timings):
        
    while supervisor.getTime() < time:
        if supervisor.step(timestep) == -1:
            quit()
    if receiver1.getQueueLength()>0:
        received_data1 = receiver1.getData()
        message1 = struct.unpack("? f f ? ? ? ?",received_data1)
        #print("Message received")
        current_node1 = position_to_node[(message1[1],0,message1[2])]
        #print("Check =",position_to_node[(message[1],0,message[2])],(message[1],0,message[2]))
        graph_updation(position_to_node[(message1[1],0,message1[2])],message1[3],message1[4],message1[5],message1[6])
        message_to_map_converion(message1[1],message1[2],message1[3],message1[4],message1[5],message1[6])
        next_node1 = next_node_generator(current_node1,1)
        #print("Message =",message)
        if message1[0]:
            quit()
        #print("Map data =",global_map_dict)
        receiver1.nextPacket()
        #print("Queue lenght",receiver1.getQueueLength())
    if receiver2.getQueueLength()>0:
        received_data2 = receiver2.getData()
        message2 = struct.unpack("? f f ? ? ? ?",received_data2)
            #print("Message received")
        current_node2 = position_to_node[(message2[1],0,message2[2])]
        #print("Check =",position_to_node[(message[1],0,message[2])],(message[1],0,message[2]))
        graph_updation(position_to_node[(message2[1],0,message2[2])],message2[3],message2[4],message2[5],message2[6])
        message_to_map_converion(message2[1],message2[2],message2[3],message2[4],message2[5],message2[6])
        next_node2 = next_node_generator(current_node2,2)
        #print("Message =",message)
        if message2[0]:
            quit()
        #print("Map data =",global_map_dict)
        receiver2.nextPacket()
        #print("Queue lenght",receiver1.getQueueLength())
    if receiver3.getQueueLength()>0:
        received_data3 = receiver3.getData()
        message3 = struct.unpack("? f f ? ? ? ?",received_data3)
            #print("Message received")
        current_node3 = position_to_node[(message3[1],0,message3[2])]
        #print("Check =",position_to_node[(message[1],0,message[2])],(message[1],0,message[2]))
        graph_updation(position_to_node[(message3[1],0,message3[2])],message3[3],message3[4],message3[5],message3[6])
        message_to_map_converion(message3[1],message3[2],message3[3],message3[4],message3[5],message3[6])
        next_node3 = next_node_generator(current_node3,3)
        #print("Message =",message)
        if message3[0]:
            quit()
        #print("Map data =",global_map_dict)
        receiver3.nextPacket()
        #print("Queue lenght",receiver1.getQueueLength())
    if receiver4.getQueueLength()>0:
        received_data4 = receiver4.getData()
        message4 = struct.unpack("? f f ? ? ? ?",received_data4)
            #print("Message received")
        current_node4 = position_to_node[(message4[1],0,message4[2])]
        #print("Check =",position_to_node[(message[1],0,message[2])],(message[1],0,message[2]))
        graph_updation(position_to_node[(message4[1],0,message4[2])],message4[3],message4[4],message4[5],message4[6])
        message_to_map_converion(message4[1],message4[2],message4[3],message4[4],message4[5],message4[6])
        next_node4 = next_node_generator(current_node4,4)
        #print("Message =",message)
        if message4[0]:
            quit()
        #print("Map data =",global_map_dict)
        receiver4.nextPacket()
        #print("Queue lenght",receiver1.getQueueLength())
    
    if bool(graph):        
        #print("Graph",generate_edges(graph))
        print("Graph",graph)
        trans_field1.setSFVec3f(list(node_to_position[next_node1]))
        trans_field2.setSFVec3f(list(node_to_position[next_node2]))
        trans_field3.setSFVec3f(list(node_to_position[next_node3]))
        trans_field4.setSFVec3f(list(node_to_position[next_node4]))
        #print("Current node Robot is in =",next_node)
            
        #(x,y,z,angle) = rot_field.getSFRotation()
        #print("Current orientation =",round(x),round(y),round(z),round(angle,4))
        print("Current node =",current_node1,"Next node =",next_node1)
        rot_field1.setSFRotation(list(next_direction(current_node1,next_node1)))
        rot_field2.setSFRotation(list(next_direction(current_node2,next_node2)))
        rot_field3.setSFRotation(list(next_direction(current_node3,next_node3)))
        rot_field4.setSFRotation(list(next_direction(current_node4,next_node4)))
        robot_node1.resetPhysics()
        robot_node2.resetPhysics()
        robot_node3.resetPhysics()
        robot_node4.resetPhysics()
    
    
    
   