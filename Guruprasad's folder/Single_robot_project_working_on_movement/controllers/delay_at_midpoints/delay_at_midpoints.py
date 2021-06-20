"""delay_at_midpoints controller."""
from controller import Robot
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
time_counter = 0
flag = 0
starting_time = 0
delay_done = False

while robot.step(timeStep) != -1:
    delay_done = False
    time_counter += timeStep
    X_pos = round(gps.getValues()[1],1)
    Z_pos = round(gps.getValues()[2],1) 
    
    
    speeds[0] = max_velocity
    speeds[1] = max_velocity
    up_distance = up_sensor.getValue()
    right_distance = right_sensor.getValue()
    left_distance = left_sensor.getValue()
    down_distance = down_sensor.getValue()
    priority_list = [["up",up_distance],["right",right_distance],["left",left_distance],["down",down_distance]]
    print("X =",X_pos,"Z =",Z_pos)
    #print("distances","up:",up_distance,"right",right_distance,"left",left_distance,"down",down_distance)
    for name,value in priority_list:
        if value<0.5:
            pass
        else:
            move(name)
            break
    if (X_pos%0.5==0 and Z_pos%0.5==0) and (flag==0) and not delay_done:
        print("Robot is in midpoint of a Square")
        print("Robot at","X value =",X_pos,"Z value =",Z_pos)
        starting_time = time_counter
        print("Starting time =",starting_time)
        flag = 1
        
        
    if starting_time!=0 and flag==1:
        if time_counter<=starting_time+1000:
            print("creating delay")
            delay()
            delay_done = True
            
    if time_counter>=starting_time+1000 and starting_time!=0:
        print("exiting delay")
        flag = 0
        starting_time = 0
    #print("X divide =",X_pos%0.5==0,"Z divide =",Z_pos%0.5==0)
    #print("Flag =",flag,"Starting time =",starting_time)
    #if Z_pos==0.0:
    #    print("******************")
    #    print("Time =",time_counter)
        #print("******************")
    
    #if time_counter>=4512 and time_counter<=7512:
    #    delay()     
    
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
