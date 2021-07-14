"""mypuck_comms_only_controller controller."""

#Importing required library Robot class
from controller import Robot
#Importing struct library required to convert messages to bytes for transmission
import struct
#Creating instance of the Robot class to access the robot(E-puck)
robot = Robot()
#Getting the timestep of the world
timeStep = int(robot.getBasicTimeStep())
#Acessing and enabling required components present in the robot
camera = robot.getDevice("colour_camera")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
gps = robot.getDevice("gps")
gps.enable(timeStep)
compass = robot.getDevice("compass")
compass.enable(timeStep)
emitter = robot.getDevice("emitter")
up_sensor = robot.getDevice("up_sensor")
up_sensor.enable(timeStep)
right_sensor = robot.getDevice("right_sensor")
right_sensor.enable(timeStep)
left_sensor = robot.getDevice("left_sensor")
left_sensor.enable(timeStep)
down_sensor = robot.getDevice("down_sensor")
down_sensor.enable(timeStep)

#Declaring standandard values (Standard Right, Up, Left and Down values got from the compass)
right = (0.0, -0.0, 1.0)
up = (-0.0, 1.0, -0.0)
left = (0.0, -0.0, -1.0)
down = (0.0, -1.0, -0.0)
#Variable which turn True when target is detected/reached
done = False
#Dicticonary to store Map data
global_dict = {}

#Fucntions
def robot_orientation(compass_val,surrounding,right_value,up_value,left_value,down_value):
    '''This function checks which direction the robot is facing and returns the surrounding data with respect to actual(global) directions'''
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
    ''' Returns True if it is a Clear-path and False if there is a Wall '''
    if inp>=0.5:
        return True
    else:
        return False

#Main Infinite Loop
while robot.step(timeStep) != -1:
    #Varible to monitor timing
    timings = range(2,180,2)
    #Looping through available timings
    for index,time in enumerate(timings):
        #This While loop to holds the robot until required time is met
        while robot.getTime() < time:
            if robot.step(timeStep) == -1:
                quit()
        #Obtaining GPS values and rounding it to 1 decimal place
        X_pos = round(gps.getValues()[0],1)
        Z_pos = round(gps.getValues()[2],1)
        #print("Debug: Position :","X pos =",X_pos,"Z pos =",Z_pos)
        #Obtaining Sensor values
        right_value = right_sensor.getValue()
        up_value = up_sensor.getValue()
        left_value = left_sensor.getValue()
        down_value = down_sensor.getValue()
        #Obtaining Compass value to know the orientation of the robot
        compass_val = [round(compass.getValues()[0],1),round(compass.getValues()[1],1),round(compass.getValues()[2],1)]
        #Instamce of the recognition node
        object= camera.getRecognitionObjects()
        #Iterating through the recognition instance
        for item in object:
            #Declaring variable
            target_pos = []
            #Checking for required recognition colour
            if item.get_colors()== [1,0,0]:
                #Getting the relative position of the target from the robot
                target_pos= item.get_position()
                #Getting the type/model of the target
                target_model= item.get_model()
                #print("Debug: Relative position of target","Target_pos =",target_pos)
                x_pos_of_target = gps.getValues()[0]+target_pos[0]
                y_pos_of_target = gps.getValues()[1]+target_pos[1]
                z_pos_of_target = gps.getValues()[2]+target_pos[2]
                #Condition to check if robot is close enough to the target
                if target_pos[0]>=0.0085:
                    #Setting the done varibale to True since target is detected and robot is near the target
                    done = True
                    #print("Target is at position","X =",x_pos_of_target,"Y =",y_pos_of_target,"Z =",z_pos_of_target)
                    print("Relative_pos =",target_pos,"Model =",target_model)
        #Varibale to store data regarding Clear-path or Wall in all four directions
        surrounding = []
        #Obatain surroundings data by passign robot orientation and sensor data
        surrounding = robot_orientation(compass_val,surrounding,right_value,up_value,left_value,down_value)
        #Updating Global map data
        global_dict[(X_pos,Z_pos)] = global_dict.get((X_pos,Z_pos),surrounding)
        #Pack the message into bytes for transmission
        message = struct.pack("? f f ? ? ? ?",done,X_pos,Z_pos,surrounding[0],surrounding[1],surrounding[2],surrounding[3])
        #print("Message =",X_pos,Z_pos,surrounding[0],surrounding[1],surrounding[2],surrounding[3])
        #Sending the message
        emitter.send(message)
    #print("Global dict :",global_dict)