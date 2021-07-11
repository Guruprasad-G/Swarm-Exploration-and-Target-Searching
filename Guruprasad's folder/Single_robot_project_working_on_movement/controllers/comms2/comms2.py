"""comms2 controller."""

from controller import Robot
import struct

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

camera = robot.getDevice("colour_camera")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
gps = robot.getDevice("gps")
gps.enable(timeStep)
emitter = robot.getDevice("emitter")
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
#wheel_left = robot.getDevice("left wheel motor")
#wheel_right = robot.getDevice("right wheel motor")
#wheel_left.setPosition(float("inf"))
#wheel_right.setPosition(float("inf"))

global_dict = {}

def obstacle_finder(inp):
    ''' Returns True if it is a Clear-path and False if there is a Wall '''
    if inp>=0.5:
        return True
    else:
        return False

while robot.step(timeStep) != -1:
    timings = range(0,180,2)
    for index,time in enumerate(timings):
        while robot.getTime() < time:
            if robot.step(timeStep) == -1:
                quit()
        X_pos = round(gps.getValues()[0],1)
        Z_pos = round(gps.getValues()[2],1)
        right_value = right_sensor.getValue()
        up_value = up_sensor.getValue()
        left_value = left_sensor.getValue()
        down_value = down_sensor.getValue()
        surrounding = []
        #Call robot_orientation function that returns a list of robot surrounding data in the order [R,U,L,D]
        surrounding = [obstacle_finder(right_value),obstacle_finder(up_value),obstacle_finder(left_value),obstacle_finder(down_value)]
        #Updating Global data
        global_dict[(X_pos,Z_pos)] = global_dict.get((X_pos,Z_pos),surrounding)
        message = struct.pack("? f f ? ? ? ?",True,X_pos,Z_pos,surrounding[0],surrounding[1],surrounding[2],surrounding[3])
        print("Message =",X_pos,Z_pos,surrounding[0],surrounding[1],surrounding[2],surrounding[3])
        emitter.send(message)
    #print("Global dict :",global_dict)
    
    #wheel_left.setVelocity(0)
    #wheel_right.setVelocity(0)
