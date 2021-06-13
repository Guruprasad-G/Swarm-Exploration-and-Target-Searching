from controller import Robot
import math
import struct

trap_colour = b'\n\n\n\xff'
swamp_colour = b'\x12\x1b \xff'
exit_colour = b'\x10\xb8\x10\xff'

timestep = 32
max_velocity = 6.28

robot = Robot()

wheel_left = robot.getDevice("left wheel motor")
wheel_right = robot.getDevice("right wheel motor")

camera= robot.getDevice("camera1")
camera.enable(timestep)
camera.recognitionEnable(timestep)


colour_camera= robot.getDevice("colour_sensor")
colour_camera.enable(timestep)

emitter= robot.getDevice("emitter")

gps= robot.getDevice("gps")
gps.enable(timestep)



leftSensors = []
rightSensors = []
frontSensors = []

frontSensors.append(robot.getDevice("ps7"))
frontSensors[0].enable(timestep)
frontSensors.append(robot.getDevice("ps0"))
frontSensors[1].enable(timestep)

rightSensors.append(robot.getDevice("ps1"))
rightSensors[0].enable(timestep)
rightSensors.append(robot.getDevice("ps2"))
rightSensors[1].enable(timestep)

leftSensors.append(robot.getDevice("ps5"))
leftSensors[0].enable(timestep)
leftSensors.append(robot.getDevice("ps6"))
leftSensors[1].enable(timestep)

#        [left wheel speed, right wheel speed]
speeds = [max_velocity,max_velocity]

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

program_start = robot.getTime()

def sendMessage(robot_type, v1, v2, v3):
    message = struct.pack('i i i c', robot_type, v1, v2, v3)
    emitter.send(message)

def sendVictimMessage():
    global messageSent
    position = gps.getValues()

    if not messageSent:
        #robot type, position x cm, position z cm, victim type
        sendMessage(0, int(position[0] * 100), int(position[2] * 100), b'H')
        messageSent = True

def getObjectDistance(position):
    return math.sqrt((position[0] ** 2) + (position[2] ** 2))

def nearObject(position):
    return getObjectDistance(position)  < 0.10


def getVisibleVictims():
    #get all objects that camera can see
    objects= camera.getRecognitionObjects()

    victims=[]

    for item in objects:
        if item.get_colors()== [1,1,1]:
            victim_pos= item.get_position()
            victims.append(victim_pos)
    return victims



def turnToVictim(victim):
    # [x,y]
    position_on_image = victim[1]

    width = camera.getWidth()
    center = width / 2

    victim_x_position = position_on_image[0]
    dx = center - victim_x_position

    if dx < 0:
        turn_right_to_victim()
    else:
        turn_left_to_victim()


def getClosestVictim(victims):
    shortestDistance = 999
    closestVictim = []

    for victim in victims:
        dist = getObjectDistance(victim[0])
        if dist < shortestDistance:
            shortestDistance = dist
            closestVictim = victim

    return closestVictim

def stopAtVictim():
    global messageSent
    #get all the victims the camera can see
    victims = getVisibleVictims()

    foundVictim = False

    if len(victims) != 0:
        closest_victim = getClosestVictim(victims)
        turnToVictim(closest_victim)

    #if we are near a victim, stop and send a message to the supervisor
    for victim in victims:
        if nearObject(victim[0]):
            stop()
            sendVictimMessage()
            foundVictim = True

    if not foundVictim:
        messageSent = False

def avoidTiles():
    global duration, startTime
    colour = colour_camera.getImage()

    if colour == trap_colour or colour == swamp_colour:
        move_backwards()
        startTime = robot.getTime()
        duration = 2

def turn_right_to_victim():
    #set left wheel speed
    speeds[0] = 1 * max_velocity
    #set right wheel speed
    speeds[1] = 0.8 * max_velocity

def turn_left_to_victim():
    #set left wheel speed
    speeds[0] = 0.8 * max_velocity
    #set right wheel speed
    speeds[1] = 1 * max_velocity

def move_backwards():
    #set left wheel speed
    speeds[0] = -0.5 * max_velocity
    #set right wheel speed
    speeds[1] = -0.7 * max_velocity

def stop():
    #set left wheel speed
    speeds[0] = 0
    #set right wheel speed
    speeds[1] = 0


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

while robot.step(timestep) != -1:
    speeds[0] = max_velocity
    speeds[1] = max_velocity

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
    stopAtVictim()
    
    
    avoidTiles()
    
    if (robot.getTime() - program_start) > 20:
         if colour_camera.getImage() == exit_colour:
               sendMessage(0,0,0,b'E')

    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])
