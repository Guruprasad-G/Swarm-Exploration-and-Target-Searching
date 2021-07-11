"""step_wise controller."""

from controller import Supervisor

TIME_STEP = 32

supervisor = Supervisor()

robot_node = supervisor.getFromDef("EPUCK1")
trans_field = robot_node.getField("translation")

t = supervisor.getTime()
timings = [2, 4, 6, 8, 10, 12]
positions = [[1.5, 0, -0.5],[2.5,0,-0.5],[2.5,0,-1.5],[3.5, 0, -1.5],[4.5,0,-1.5],[4.5,0,-2.5]]

for index,time in enumerate(timings): 
    while supervisor.getTime() - t < time:
        if supervisor.step(TIME_STEP) == -1:
            quit()
    trans_field.setSFVec3f(positions[index])
    robot_node.resetPhysics()