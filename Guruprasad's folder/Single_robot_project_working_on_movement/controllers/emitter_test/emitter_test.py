"""emitter_test controller."""

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
emitter = robot.getDevice("emitter")
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

def turn_right():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.2 * max_velocity

def turn_left():
    #set left wheel speed
    speeds[0] = -0.2 * max_velocity
    #set right wheel speed
    speeds[1] = 0.6 * max_velocity

def spin():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.6 * max_velocity

def delay():
    speeds[0] = 0
    speeds[1] = 0

def move(name):
    if name=="up":
        speeds[0] = 0.9 * max_velocity
        speeds[1] = 0.9 * max_velocity
    elif name=="right":
        turn_right()
    elif name=="left":
        turn_left()
    else:
        pass
def obstacle_finder(inp):
    if inp>0.5:
        return "clear path"
    else:
        return "wall"
lst = [5120,6.28*32*2,6.28*32*3,6.28*32*4]
time_counter = 0
starting_time = 0
counter_dict = {}
counter_dict = {0.5:0,0.4:0,0.3:0,0.2:0,0.1:0,0.0:0,-0.1:0,-0.2:0,-0.3:0,-0.4:0,-0.5:0,-0.6:0,-0.7:0,-0.8:0,-0.9:0,-1:0,
                -1.1:0,-1.2:0,-1.3:0,-1.4:0,-1.5:0,-1.6:0,-1.7:0,-1.8:0,-1.9:0,-2.0:0,-2.1:0,-2.2:0,-2.3:0,-2.4:0,-2.5:0}
global_dict = {}
while robot.step(timeStep) != -1:
    #surroundings = []
    time_counter += timeStep
    X_pos = round(gps.getValues()[1],1)
    Z_pos = round(gps.getValues()[2],1) 
    if str(Z_pos)[-1]=='5':
        right_value = right_sensor.getValue()
        up_value = up_sensor.getValue()
        left_value = left_sensor.getValue()
        down_value = down_sensor.getValue()
        surrounding = [obstacle_finder(right_value),obstacle_finder(up_value),
                       obstacle_finder(left_value),obstacle_finder(down_value)]
        global_dict[(X_pos,Z_pos)] = global_dict.get((X_pos,Z_pos),surrounding)
        print("global_dict :",global_dict)
        if "wall" in global_dict[(X_pos,Z_pos)]:
            message = struct.pack("i",1)
            emitter.send(message)
    
    speeds[0] = max_velocity
    speeds[1] = max_velocity
    up_distance = up_sensor.getValue()
    right_distance = right_sensor.getValue()
    left_distance = left_sensor.getValue()
    down_distance = down_sensor.getValue()
    priority_list = [["up",up_distance],["right",right_distance],["left",left_distance],["down",down_distance]]
    #print("X =",X_pos,"Z =",Z_pos)
    
    
    
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
        
    
    if up_distance<0.5:
        pass
        #print("Up Distance is less than 0.5")
    
    if right_distance<0.5:
        pass
        #turn_left()
        #print("Right Distance is less than 0.5")
    
    if left_distance<0.5:
        pass
        #turn_right()
        #print("Left Distance is less than 0.5")
    
    if down_distance<0.5:
        pass
        #print("Down Distance is less than 0.5")

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