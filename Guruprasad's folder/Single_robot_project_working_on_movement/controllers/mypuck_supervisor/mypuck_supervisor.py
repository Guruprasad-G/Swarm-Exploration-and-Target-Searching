"""mypuck_supervisor controller."""

#Importing the Supervisor library
from controller import Supervisor
#Importing struct library required to convert bytes to message on reception of message
import struct
#Creating instance of the supervisor class
supervisor = Supervisor()
#Getting the timestep of the world
timestep = int(supervisor.getBasicTimeStep())
#Creating access to each of the four E-pucks
robot_node1 = supervisor.getFromDef("EPUCK1")
robot_node2 = supervisor.getFromDef("EPUCK2")
robot_node3 = supervisor.getFromDef("EPUCK3")
robot_node4 = supervisor.getFromDef("EPUCK4")
#Acessing and enabling required components present in the supervisor
receiver1 = supervisor.getDevice("receiver1")
receiver1.enable(timestep)
receiver2 = supervisor.getDevice("receiver2")
receiver2.enable(timestep)
receiver3 = supervisor.getDevice("receiver3")
receiver3.enable(timestep)
receiver4 = supervisor.getDevice("receiver4")
receiver4.enable(timestep)

#Getting access to each of the robot's translation and rotation
trans_field1 = robot_node1.getField("translation")
rot_field1= robot_node1.getField("rotation")
trans_field2 = robot_node2.getField("translation")
rot_field2= robot_node2.getField("rotation")
trans_field3 = robot_node3.getField("translation")
rot_field3= robot_node3.getField("rotation")
trans_field4 = robot_node4.getField("translation")
rot_field4= robot_node4.getField("rotation")

#Declaring standandard values (Standard Right, Up, Left and Down values got from the rotation values)
right = (0,1,0,-1.57071) 
up = (0,1,0,0)
left = (0,1,0,1.57071)
down = (0,1,0,3.14142)
#Length of the Arena leaving the buffer zone
length_of_arena = 8
#Varibles to monitor current and next nodes of the robots
next_node1 = 0
current_node1 = 0
next_node2 = 0
current_node2 = 0
next_node3 = 0
current_node3 = 0
next_node4 = 0
current_node4 = 0

#Dictionary containing Map data collected by each of the robots
global_map_dict = {}
#A list that is used to mark the nodes that are visited by the robots
visited = [False for i in range(64)]

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

#Creating instances pf Stack for each of the robot
stack1 = Stack()
stack2 = Stack()
stack3 = Stack()
stack4 = Stack()
#Dictionary that corresponds robot to their respective stack
stack_access_dict = {1:stack1,2:stack2,3:stack3,4:stack4}

#Graph code from GeeksForGeeks
  
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

#Functions
def next_node_generator(current_node,robot_number):
    '''This function updates the nodes visited by a robot and checks the possible path it can go.'''
    '''It also keeps a track of nodes visited for backtracking in case of deadends'''
    global visited
    global graph
    global stack_access_dict
    #Add current node to corresponding robot stack
    stack_access_dict[robot_number].push(current_node)
    #Update current node as visited
    visited[current_node] = True
    #Check the connections for current node
    for node in graph[current_node]:
        #If one of the connected node is not visited, return it
        if (not visited[node]):
            #print("Debug: ","Node to be returned =",node)
            return node
    #If all connected nodes are visited, pop out current node from stack
    stack_access_dict[robot_number].pop()
    #Return the node that was pushed into the stack before the current node
    #print("Debug: ","Back path to be returned =",stack.peek())
    return stack_access_dict[robot_number].pop()

def message_to_map_converion(X_pos,Z_pos,r_detail,u_detail,l_detail,d_detail):
    '''This function updates the Global dictionary containing map data based on message recived from the robots'''
    global global_map_dict
    surrounding = [r_detail,u_detail,l_detail,d_detail]
    global_map_dict[(X_pos,Z_pos)] = global_map_dict.get((X_pos,Z_pos),surrounding)

def graph_updation(current_node,r,u,l,d):
    '''This function updates the nodes and their connection to graph data structure based on message received from the robots'''
    global graph
    if u and (current_node-8 not in graph[current_node]):
        addEdge(graph,current_node,current_node-8)
    if l and (current_node-1 not in graph[current_node]):
        addEdge(graph,current_node,current_node-1)
    if r and (current_node+1 not in graph[current_node]):
        addEdge(graph,current_node,current_node+1)
    if d and (current_node+8 not in graph[current_node]):
        addEdge(graph,current_node,current_node+8)

def next_direction(current_node,next_node):
    '''This function returns the orientation in which the robot has to be rotated based on current and next node'''
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
    '''This function returns two dictinaries with key and values as node and position values in the arena and vice versa'''
    if division==True:
        length_of_arena = int(length_of_arena/2)
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
    for i in range(0,pow(length_of_arena,3)):
        node_to_position[i] = node_to_position.get(i,mid_positions[i])
    position_to_node = dict([(value, key) for key, value in node_to_position.items()])
    return node_to_position,position_to_node

#Dictinonary obtained to relate between Nodes and Positions
(node_to_position,position_to_node) = conversion_between_node_and_position(length_of_arena)
print("Debug :","Node <=> Positions",node_to_position,"\n",position_to_node)
#nodes = pow(length_of_arena,3)
#Varible to monitor timing    
timings = range(2,180,2)

#Looping through available timings
for index,time in enumerate(timings):
    #This While loop to holds the supervisor until required time is met    
    while supervisor.getTime() < time:
        if supervisor.step(timestep) == -1:
            quit()
    #Code to receive messages from MYPUCK1
    if receiver1.getQueueLength()>0:
        received_data1 = receiver1.getData()
        message1 = struct.unpack("? f f ? ? ? ?",received_data1)
        #Check which node the robot is present at based on input message about the robot's position
        current_node1 = position_to_node[(message1[1],0,message1[2])]
        #print("Debug : Received Message =",message1)
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message1[1],0,message1[2])],message1[3],message1[4],message1[5],message1[6])
        #Updating the Global map data
        message_to_map_converion(message1[1],message1[2],message1[3],message1[4],message1[5],message1[6])
        #print("Debug :",Map data =",global_map_dict)
        #Calling function to receive next node the robot has to translate to
        next_node1 = next_node_generator(current_node1,1)
        if message1[0]:
            #Stop supervisor if target is found and reached
            quit()
        receiver1.nextPacket()
        #print("Debug :","Queue lenght",receiver1.getQueueLength())
    
    #Code to receive messages from MYPUCK2
    if receiver2.getQueueLength()>0:
        received_data2 = receiver2.getData()
        message2 = struct.unpack("? f f ? ? ? ?",received_data2)
        #Check which node the robot is present at based on input message about the robot's position
        current_node2 = position_to_node[(message2[1],0,message2[2])]
        #print("Debug : Received Message =",message2)
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message2[1],0,message2[2])],message2[3],message2[4],message2[5],message2[6])
        #Updating the Global map data
        message_to_map_converion(message2[1],message2[2],message2[3],message2[4],message2[5],message2[6])
        #print("Debug :",Map data =",global_map_dict)
        #Calling function to receive next node the robot has to translate to
        next_node2 = next_node_generator(current_node2,2)
        if message2[0]:
            #Stop supervisor if target is found and reached
            quit()
        receiver2.nextPacket()
        #print("Debug :","Queue lenght",receiver2.getQueueLength())
    
    #Code to receive messages from MYPUCK3
    if receiver3.getQueueLength()>0:
        received_data3 = receiver3.getData()
        message3 = struct.unpack("? f f ? ? ? ?",received_data3)
        #Check which node the robot is present at based on input message about the robot's position
        current_node3 = position_to_node[(message3[1],0,message3[2])]
        #print("Debug : Received Message =",message3)
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message3[1],0,message3[2])],message3[3],message3[4],message3[5],message3[6])
        #Updating the Global map data
        message_to_map_converion(message3[1],message3[2],message3[3],message3[4],message3[5],message3[6])
        #print("Debug :",Map data =",global_map_dict)
        #Calling function to receive next node the robot has to translate to
        next_node3 = next_node_generator(current_node3,3)
        if message3[0]:
            #Stop supervisor if target is found and reached
            quit()
        receiver3.nextPacket()
        #print("Debug :","Queue lenght",receiver3.getQueueLength())
    
    #Code to receive messages from MYPUCK4
    if receiver4.getQueueLength()>0:
        received_data4 = receiver4.getData()
        message4 = struct.unpack("? f f ? ? ? ?",received_data4)
        #Check which node the robot is present at based on input message about the robot's position
        current_node4 = position_to_node[(message4[1],0,message4[2])]
        #print("Debug : Received Message =",message4)
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message4[1],0,message4[2])],message4[3],message4[4],message4[5],message4[6])
        #Updating the Global map data
        message_to_map_converion(message4[1],message4[2],message4[3],message4[4],message4[5],message4[6])
        #print("Debug :",Map data =",global_map_dict)
        #Calling function to receive next node the robot has to translate to
        next_node4 = next_node_generator(current_node4,4)
        if message4[0]:
            #Stop supervisor if target is found and reached
            quit()
        receiver4.nextPacket()
        #print("Debug :","Queue lenght",receiver4.getQueueLength())
    
    #Start changing the postions of robots only when the graph of nodes is not empty
    if bool(graph):        
        #print("Debug :","Graph =",graph)

        #Debug/Monitor statements for each of the robots
        #print("Debug : First robot","Current node =",current_node1,"Next node =",next_node1)
        #print("Debug : Second robot","Current node =",current_node2,"Next node =",next_node2)
        #print("Debug : Third robot","Current node =",current_node3,"Next node =",next_node3)
        #print("Debug : Fourth robot","Current node =",current_node4,"Next node =",next_node4)

        #Setting the robots to their respective next node
        trans_field1.setSFVec3f(list(node_to_position[next_node1]))
        trans_field2.setSFVec3f(list(node_to_position[next_node2]))
        trans_field3.setSFVec3f(list(node_to_position[next_node3]))
        trans_field4.setSFVec3f(list(node_to_position[next_node4]))
        
        #Setting the robot's orientation based on position of next node
        rot_field1.setSFRotation(list(next_direction(current_node1,next_node1)))
        rot_field2.setSFRotation(list(next_direction(current_node2,next_node2)))
        rot_field3.setSFRotation(list(next_direction(current_node3,next_node3)))
        rot_field4.setSFRotation(list(next_direction(current_node4,next_node4)))
        
        #Resetting Physics dependencies for each of the robots so that it doesn't behave in unexpected ways
        robot_node1.resetPhysics()
        robot_node2.resetPhysics()
        robot_node3.resetPhysics()
        robot_node4.resetPhysics()
        