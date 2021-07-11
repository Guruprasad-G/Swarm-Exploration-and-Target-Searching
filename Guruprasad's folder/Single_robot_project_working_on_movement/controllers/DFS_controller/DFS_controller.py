"""DFS_controller controller."""

from controller import Supervisor

TIME_STEP = 32

supervisor = Supervisor()

def node_to_position_conversion(length_of_arena,division=True):
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
    return node_to_position

while supervisor.step(timestep) != -1:
    node_to_position = node_to_position_conversion(4)
    #print(node_to_position)


