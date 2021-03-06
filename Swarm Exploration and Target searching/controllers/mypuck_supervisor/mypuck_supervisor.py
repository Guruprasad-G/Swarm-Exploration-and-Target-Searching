"""mypuck_supervisor controller."""

#Importing the Supervisor library
from controller import Supervisor
#Importing struct library required to convert bytes to message on reception of message
import struct
# import dictionary for graph
from collections import defaultdict
#Importing python script to generate map
import map_generator
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
homenode1 = 0
homenode2 = 0
homenode3 = 0
homenode4 =0
first_run = [True,True,True,True]
flag = False

#Dictionary containing Map data collected by each of the robots
global_map_dict = {}
#A list that is used to mark the nodes that are visited by the robots
visited = [False for i in range(length_of_arena*length_of_arena)]
#print("Debug: Visited =",visited,"Length of visited =",len(visited))

#Code for implementing the Stack data structure
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
def next_node_generator(current_node,robot_number,conflict_avoider=[]):
    '''This function performs DFS operation by updating the nodes visited by a robot and checking the possible path it can go.'''
    '''It also keeps a track of nodes visited for backtracking in case of deadends'''
    global visited
    global graph
    global stack_access_dict
    global homenode1
    global homenode2
    global homenode3
    global homenode4
    #Add current node to corresponding robot stack
    stack_access_dict[robot_number].push(current_node)
    #Check the connections for current node
    for node in graph[current_node]:
        #If one of the connected node is not visited, return it
        #print("Debug: Node =",node)
        if (not visited[node]) and (node not in conflict_avoider):
            #print("Debug: ","Node to be returned =",node)
            if current_node == node:
                break
            return node
    if current_node in [homenode1,homenode2,homenode3,homenode4]:
        return current_node
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

def write_map_to_file(target_pos):
    '''This function writes the data stored in the Global Map data to a text file'''
    global global_map_dict
    file_handle = open("Map_details.txt", "w")
    for key,value in global_map_dict.items():
        sentence = "Position,{},{},Data,{},{},{},{}\n".format(str(key[0]),str(key[1]),str(value[0]),str(value[1]),str(value[2]),str(value[3]))
        file_handle.write(sentence)
    target_pos = (target_pos[0],target_pos[2])
    target_location = "Position,{},{},Data,{},{},{},{}\n".format(str(target_pos[0]),str(target_pos[1]),str(global_map_dict[target_pos][0]),str(global_map_dict[target_pos][1]),str(global_map_dict[target_pos][2]),str(global_map_dict[target_pos][3]))
    file_handle.close()

def graph_updation(current_node,r,u,l,d):
    '''This function updates the nodes and their connection to graph data structure based on message received from the robots'''
    global length_of_arena
    global graph
    if u and (current_node-length_of_arena not in graph[current_node]) and (current_node-length_of_arena>=0):
        addEdge(graph,current_node,current_node-length_of_arena)
    if l and (current_node-1 not in graph[current_node]) and (current_node-1>=0):
        addEdge(graph,current_node,current_node-1)
    if r and (current_node+1 not in graph[current_node]):
        addEdge(graph,current_node,current_node+1)
    if d and (current_node+length_of_arena not in graph[current_node]):
        addEdge(graph,current_node,current_node+length_of_arena)

def next_direction(current_node,next_node):
    '''This function returns the orientation in which the robot has to be rotated based on current and next node'''
    global length_of_arena
    global right
    global up
    global left
    global down
    if current_node+1 == next_node:
        return right
    elif current_node-length_of_arena == next_node:
        return up
    elif current_node-1 == next_node:
        return left
    elif current_node+length_of_arena == next_node:
        return down
    elif current_node==next_node:
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
    for i in range(0,2*length_of_arena*2*length_of_arena):
        node_to_position[i] = node_to_position.get(i,mid_positions[i])
    position_to_node = dict([(value, key) for key, value in node_to_position.items()])
    return node_to_position,position_to_node

#Dictinonary obtained to relate between Nodes and Positions
(node_to_position,position_to_node) = conversion_between_node_and_position(length_of_arena)
#print("Debug :","Node <=> Positions",node_to_position,"\n",position_to_node)

#Varible to monitor timing    
timings = range(1,720)

#Looping through available timings
for time in timings:
    #This While loop to holds the supervisor until required time is met    
    conflict_avoider = []
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
        if first_run[0] and bool(graph):
            homenode1 = current_node1
            print("Homenode1 =",homenode1)
            first_run[0] = False
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message1[1],0,message1[2])],message1[3],message1[4],message1[5],message1[6])
        #Updating the Global map data
        message_to_map_converion(message1[1],message1[2],message1[3],message1[4],message1[5],message1[6])
        #print("Debug :",Map data =",global_map_dict)
        #Update current node as visited
        visited[current_node1] = True
        receiver1.nextPacket()
        #print("Debug :","Queue lenght",receiver1.getQueueLength())
    
    #Code to receive messages from MYPUCK2
    if receiver2.getQueueLength()>0:
        received_data2 = receiver2.getData()
        message2 = struct.unpack("? f f ? ? ? ?",received_data2)
        #Check which node the robot is present at based on input message about the robot's position
        current_node2 = position_to_node[(message2[1],0,message2[2])]
        #print("Debug : Received Message =",message2)
        if first_run[1] and bool(graph):
            homenode2 = current_node2
            print("Homenode2 =",homenode2)
            first_run[1] = False
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message2[1],0,message2[2])],message2[3],message2[4],message2[5],message2[6])
        #Updating the Global map data
        message_to_map_converion(message2[1],message2[2],message2[3],message2[4],message2[5],message2[6])
        #print("Debug :",Map data =",global_map_dict)
        visited[current_node2] = True
        receiver2.nextPacket()
        #print("Debug :","Queue lenght",receiver2.getQueueLength())
    
    #Code to receive messages from MYPUCK3
    if receiver3.getQueueLength()>0:
        received_data3 = receiver3.getData()
        message3 = struct.unpack("? f f ? ? ? ?",received_data3)
        #Check which node the robot is present at based on input message about the robot's position
        current_node3 = position_to_node[(message3[1],0,message3[2])]
        #print("Debug : Received Message =",message3)
        if first_run[2] and bool(graph):
            homenode3 = current_node3
            print("Homenode3 =",homenode3)
            first_run[2] = False
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message3[1],0,message3[2])],message3[3],message3[4],message3[5],message3[6])
        #Updating the Global map data
        message_to_map_converion(message3[1],message3[2],message3[3],message3[4],message3[5],message3[6])
        #print("Debug :",Map data =",global_map_dict)
        visited[current_node3] = True
        receiver3.nextPacket()
        #print("Debug :","Queue lenght",receiver3.getQueueLength())
    
    #Code to receive messages from MYPUCK4
    if receiver4.getQueueLength()>0:
        received_data4 = receiver4.getData()
        message4 = struct.unpack("? f f ? ? ? ?",received_data4)
        #Check which node the robot is present at based on input message about the robot's position
        current_node4 = position_to_node[(message4[1],0,message4[2])]
        #print("Debug : Received Message =",message4)
        if first_run[3] and bool(graph):
            homenode4 = current_node4
            print("Homenode4 =",homenode4)
            first_run[3] = False
        #Update the graph for connection between nodes based on the received message
        graph_updation(position_to_node[(message4[1],0,message4[2])],message4[3],message4[4],message4[5],message4[6])
        #Updating the Global map data
        message_to_map_converion(message4[1],message4[2],message4[3],message4[4],message4[5],message4[6])
        #print("Debug :",Map data =",global_map_dict)
        visited[current_node4] = True
        receiver4.nextPacket()
        #print("Debug :","Queue lenght",receiver4.getQueueLength())
    
    #Start changing the postions of robots only when the graph of nodes is not empty
    if bool(graph):        
        #print("Debug :","Graph =",graph)
        #print("Debug :","Global map dict =",global_map_dict)
        
        #Calling function to receive next node the robot has to translate to
        next_node1 = next_node_generator(current_node1,1)
        conflict_avoider.append(next_node1)
        next_node2 = next_node_generator(current_node2,2,conflict_avoider)
        conflict_avoider.append(next_node2)
        next_node3 = next_node_generator(current_node3,3,conflict_avoider)
        conflict_avoider.append(next_node3)
        next_node4 = next_node_generator(current_node4,4,conflict_avoider)
        
        #Debug/Monitor statements for each of the robots
        #print("Debug : First robot","Current node =",current_node1,"Next node =",next_node1)
        #print("Debug : Second robot","Current node =",current_node2,"Next node =",next_node2)
        #print("Debug : Third robot","Current node =",current_node3,"Next node =",next_node3)
        #print("Debug : Fourth robot","Current node =",current_node4,"Next node =",next_node4)
        
        for index,message in enumerate([message1[0],message2[0],message3[0],message4[0]]):
            if message:
                message_to_current_node_dict = {1:current_node1,2:current_node2,3:current_node3,4:current_node4}
                target_pos = node_to_position[message_to_current_node_dict[index+1]]
                target_val = global_map_dict[(target_pos[0],target_pos[2])]
                del global_map_dict[(target_pos[0],target_pos[2])]
                global_map_dict[(target_pos[0],target_pos[2])] = target_val
                #Write the Global Map data to a test file
                write_map_to_file(node_to_position[message_to_current_node_dict[index+1]])
                #Stop supervisor if target is found and reached
                #map_generator.generate_map(global_map_dict)
                map_generator.generate_map()
                supervisor.simulationSetMode(0)
                supervisor.simulationReset()
                exit()
        
        if current_node1 == homenode1 and flag and time>4:
            print("Robot 1 has explored as much as it could and has returned to it's starting position")
        if current_node2 == homenode2 and flag:
            print("Robot 2 has explored as much as it could and has returned to it's starting position")
        if current_node3 == homenode3 and flag:
            print("Robot 3 has explored as much as it could and has returned to it's starting position")
        if current_node4 == homenode4 and flag:
            print("Robot 4 has explored as much as it could and has returned to it's starting position")
        if (True not in first_run):
            flag = True
        
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
        