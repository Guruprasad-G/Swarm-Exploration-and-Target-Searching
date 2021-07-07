"""making_turns controller."""

from controller import Robot
import struct
#declaring timestep and maximum velocity.
timeStep = 32
max_velocity = 6.28
#creating instance of robot class
robot = Robot()
#accessing and enabling required components in the robot
camera = robot.getDevice("colour_camera")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
gps = robot.getDevice("gps")
gps.enable(timeStep)
compass = robot.getDevice("compass")
compass.enable(timeStep)
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)
up_sensor = robot.getDevice("up_sensor")
up_sensor.enable(timeStep)
right_sensor = robot.getDevice("right_sensor")
right_sensor.enable(timeStep)
left_sensor = robot.getDevice("left_sensor")
left_sensor.enable(timeStep)
down_sensor = robot.getDevice("down_sensor")
down_sensor.enable(timeStep)
wheel_left = robot.getDevice("left wheel motor")
wheel_right = robot.getDevice("right wheel motor")

#Appending In-Built Sensors ---- Based on Erebus Rescue Sim
leftSensors = []
rightSensors = []
frontSensors = []
frontSensors.append(robot.getDevice("ps7"))
frontSensors[0].enable(timeStep)
frontSensors.append(robot.getDevice("ps0"))
frontSensors[1].enable(timeStep)
rightSensors.append(robot.getDevice("ps1"))
rightSensors[0].enable(timeStep)
rightSensors.append(robot.getDevice("ps2"))
rightSensors[1].enable(timeStep)
leftSensors.append(robot.getDevice("ps5"))
leftSensors[0].enable(timeStep)
leftSensors.append(robot.getDevice("ps6"))
leftSensors[1].enable(timeStep)

#[left wheel speed, right wheel speed]
speeds = [max_velocity,max_velocity]
wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

#Initializing Required Variables
current_dir = []
global_dict = {}
time_counter = 0

#Declaring Standard Variables
right = (0.0, -0.0, 1.0)
up = (-0.0, 1.0, -0.0)
left = (0.0, -0.0, -1.0)
down = (0.0, -1.0, -0.0)
direction_dict = {"right":right,"up":up,"left":left,"down":down}
left_next_dir =   {right:list(up), up:list(left), left:list(down), down:list(right)}
right_next_dir =  {right:list(down), down:list(left), left:list(up), up:list(right)}
rotate_next_dir = {right:list(left), left:list(right), up:list(down), down:list(up)}

#Varibales required for Mid-point delay -------Still in testing, can be Bypassed 
lst = [5120,6.28*32*2,6.28*32*3,6.28*32*4]
starting_time = 0
counter_dict = {0.5:0,0.4:0,0.3:0,0.2:0,0.1:0,0.0:0,-0.1:0,-0.2:0,-0.3:0,-0.4:0,-0.5:0,-0.6:0,-0.7:0,-0.8:0,-0.9:0,-1:0,
                -1.1:0,-1.2:0,-1.3:0,-1.4:0,-1.5:0,-1.6:0,-1.7:0,-1.8:0,-1.9:0,-2.0:0,-2.1:0,-2.2:0,-2.3:0,-2.4:0,-2.5:0}

#Functions
def robot_orientation(compass_val,surrounding,right_value,up_value,left_value,down_value):
    global right
    global up
    global left
    global down
    if tuple(compass_val) == up:
        surrounding = [obstacle_finder(right_value),obstacle_finder(up_value),
                       obstacle_finder(left_value),obstacle_finder(down_value)]
    elif tuple(compass_val) == right:
        surrounding = [obstacle_finder(up_value),obstacle_finder(left_value),
                       obstacle_finder(down_value),obstacle_finder(right_value)]
    elif tuple(compass_val) == left:
        surrounding = [obstacle_finder(down_value),obstacle_finder(right_value),
                       obstacle_finder(up_value),obstacle_finder(left_value)]
    elif tuple(compass_val) == down:
        surrounding = [obstacle_finder(left_value),obstacle_finder(down_value),
                       obstacle_finder(right_value),obstacle_finder(up_value)]
    else :
        print("Compass Value Error : Invalid Compass value inside robot_orientation function")
    return surrounding

def obstacle_finder(inp):
    if inp>=0.5:
        return "clear path"
    else:
        return "wall"

def movement_decision(X_pos,Z_pos):
    global global_dict
    global current_dir
    global right
    global up
    global left
    global down
    if tuple(current_dir) == right:
        if global_dict[(X_pos,Z_pos)][0] == "wall":
            if global_dict[(X_pos,Z_pos)][3] == "clear path":
                return "turn_right"
            elif global_dict[(X_pos,Z_pos)][1] == "clear path":
                return "turn_left"
            elif global_dict[(X_pos,Z_pos)][2] == "clear path":
                return "turn_back"
            else:
                print("Hey I am Surrounded on all the sides man!!")
                return "delay"
        else:
            return "forward"
    elif tuple(current_dir) == up:
        if global_dict[(X_pos,Z_pos)][1] == "wall":
            if global_dict[(X_pos,Z_pos)][0] == "clear path":
                return "turn_right"
            elif global_dict[(X_pos,Z_pos)][2] == "clear path":
                return "turn_left"
            elif global_dict[(X_pos,Z_pos)][3] == "clear path":
                return "turn_back"
            else:
                print("Hey I am Surrounded on all the sides man!!")
                return "delay"
        else:
            return "forward"    
    elif tuple(current_dir) == left:
        if global_dict[(X_pos,Z_pos)][2] == "wall":
            if global_dict[(X_pos,Z_pos)][1] == "clear path":
                return "turn_right"
            elif global_dict[(X_pos,Z_pos)][3] == "clear path":
                return "turn_left"
            elif global_dict[(X_pos,Z_pos)][0] == "clear path":
                return "turn_back"
            else:
                print("Hey I am Surrounded on all the sides man!!")
                return "delay"
        else:
            return "forward"
    elif tuple(current_dir) == down:
        if global_dict[(X_pos,Z_pos)][3] == "wall":
            if global_dict[(X_pos,Z_pos)][2] == "clear path":
                return "turn_right"
            elif global_dict[(X_pos,Z_pos)][0] == "clear path":
                return "turn_left"
            elif global_dict[(X_pos,Z_pos)][1] == "clear path":
                return "turn_back"
            else:
                print("Hey I am Surrounded on all the sides man!!")
                return "delay"
        else:
            return "forward"
    else :
        return "forward"

def move(name):
    global current_dir
    if name=="forward":
        speeds[0] = max_velocity
        speeds[1] = max_velocity
    elif name=="turn_right":
        turn_right()
    elif name=="turn_left":
        turn_left()
    elif name=="turn_back":
        spin()
    else:
        delay()

def turn_right():
    global current_dir
    global right_next_dir
    #print("Current",current_dir,"Check",tuple(current_dir))
    if(current_dir!=right_next_dir[tuple(current_dir)]):
        print("Current dir =",current_dir,"End dir =",right_next_dir[tuple(current_dir)])
        #set left wheel speed
        speeds[0] = 0.6 * max_velocity
        #set right wheel speed
        speeds[1] = -0.1 * max_velocity

def turn_left():
    global current_dir
    global left_next_dir  
    if(current_dir!=left_next_dir[tuple(current_dir)]):    
        #set left wheel speed
        speeds[0] = -0.1 * max_velocity
        #set right wheel speed
        speeds[1] = 0.6 * max_velocity

def spin():
    global current_dir
    global rotate_next_dir
    if(current_dir!=rotate_next_dir[tuple(current_dir)]): 
        #set left wheel speed
        speeds[0] = 0.5 * max_velocity
        #set right wheel speed
        speeds[1] = -0.5 * max_velocity

def delay():
    speeds[0] = 0
    speeds[1] = 0

#Main loop which executed at every time step
while robot.step(timeStep) != -1:
    #Incrementing Time counter to get current simulation time
    time_counter += timeStep
    
    #To get approximate GPS position of the robot
    X_pos = round(gps.getValues()[0],1)
    Z_pos = round(gps.getValues()[2],1) 
    print("X =",X_pos,"Z =",Z_pos)
    
    #By default the robot keeps moving forward
    speeds[0] = max_velocity
    speeds[1] = max_velocity
    
    #The folowing gets executed when the robot is at the approximate center of the square
    if str(Z_pos)[-1]=='5' and str(X_pos)[-1]=='5':
        #To get the sensor values
        right_value = right_sensor.getValue()
        up_value = up_sensor.getValue()
        left_value = left_sensor.getValue()
        down_value = down_sensor.getValue()
        
        #Get compass value to check robot orientatio
        compass_val = [round(compass.getValues()[0],1),round(compass.getValues()[1],1),round(compass.getValues()[2],1)]
        print("Compass Value :",compass_val)
        
        #Updates current direction and global dict value only when compass value is in right,up,left or down
        if tuple(compass_val) in [right,up,left,down]:
            surrounding = []
            #Call robot_orientation function that returns a list of robot surrounding data in the order [R,U,L,D]
            surrounding = robot_orientation(compass_val,surrounding,right_value,up_value,left_value,down_value)
            #Updating Global data
            global_dict[(X_pos,Z_pos)] = global_dict.get((X_pos,Z_pos),surrounding)
            print("Global dict :",global_dict)
            print("Current dir set to",compass_val)
            current_dir = compass_val
          
        #Calls movement decision which returns how to move about 
        move(movement_decision(X_pos,Z_pos))
     
    #Receives message only when Message Queue is greater than 0
    if receiver.getQueueLength()>0:
        #Webots way of reciveing message
        message=receiver.getData()
        #Message needs to be unpacked to recover orginal message
        dataList=struct.unpack("i",message)
        print("Message =",dataList)
        if 1 in dataList:
            print("Second robot has detected wall on it's right side -- by First robot")
    
    #Initializing CameraRecognitionObject
    object= camera.getRecognitionObjects()
    #print(obj)
    for item in object:
        target_pos = []
        if item.get_colors()== [1,0,0]:
            victim_pos= item.get_position()
            victim_model= item.get_model()
            #print("Target_pos =",target_pos)
            x_pos_of_target = gps.getValues()[0]+victim_pos[0]
            y_pos_of_target = gps.getValues()[1]+victim_pos[1]
            z_pos_of_target = gps.getValues()[2]+victim_pos[2]
            print("Relative_pos =",victim_pos)
            print("GPS pos=",gps.getValues())
            print("Target {} found at a relative distance of X={},Y={},Z={}".format(victim_model,x_pos_of_target,y_pos_of_target,z_pos_of_target))
            #print(victim_pos,victim_model)
            #print("target found")
            if victim_pos[0]>0.032  or victim_pos[2]>0.032:
               delay()
    
    
    #Code for midpoint delay -------- Need to work on it, Can be bypassed
    if time_counter in lst:
        starting_time = time_counter
        
    if time_counter<=starting_time+1000:
        pass
        #delay()
        
    if Z_pos in counter_dict:
        counter_dict[Z_pos]+=1
    
    for key,value in counter_dict.items():
        if value!=25:
            pass
            #print(key)
    #print("*******")
    #print(counter_dict)
    #print(X_pos,Z_pos) 
    
    #**********E-Puck Erebus Rescue Sim Code Copy*************#
    for i in range(2):
        #for sensors on the left, either
        if leftSensors[i].getValue() > 80:
            turn_right()
        #for sensors on the right, either
        elif rightSensors[i].getValue() > 80:
            turn_left()

    #for both front sensors
    if frontSensors[0].getValue() > 80 and frontSensors[1].getValue() > 80:
        spin()

    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])