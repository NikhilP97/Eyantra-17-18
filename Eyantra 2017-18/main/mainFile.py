# Libraries
import time
import allFunctionFiles  # all function required imported

# serial setup
port = "/dev/ttyUSB1"
baud = 9600
ser = serial.Serial(port, baud, timeout=None)

# Initializing Variables

obstacle_list = []  # List storing locations(node numbers) of obstacles

turn_cost_90 = 510  # Turn cost used for a* algorithm, obtained by measuring time it takes for bot to turn
turn_cost_180 = 700
cell_to_cell_cost = 200  # Cost for movement from cell to cell, used for a* algorithm

init_start_node = 0
init_end_node = 26
init_direction = 'N'

final_deposition_node = 0

# Deposition Zones for different fruits given as Node numbers
depostion_zone_apple = [37, 38, 44, 45]
depostion_zone_bberry = [35, 36, 42, 43]
depostion_zone_orange = [40, 41, 47, 48]

# Required Fruits Table
apple_dict = {'L': 1, 'M': 1, 'S': 1}
blue_dict = {'L': 1, 'M': 1, 'S': 1}
orange_dict = {'L': 0, 'M': 2, 'S': 0}

# Tree locations of the fruits
tree_location_1 = 9
tree_location_2 = 29
tree_location_3 = 18

# Shortest Node to DZ, used in a* algorithm
shortest_node_to_dep = 0

# nodes on which bot can travel
travesable_nodes = [

    42, 43, 44, 45, 46, 47, 48,
    35, 36, 37, 38, 39, 40, 41,
    28, 29, 30, 31, 32, 33, 34,
    21, 22, 23, 24, 25, 26, 27,
    14, 15, 16, 17, 18, 19, 20,
    07,  8,  9, 10, 11, 12, 13,
    00, 01, 02, 03, 04, 05, 06,

]

# Stores character values needed to be sent for each command. Same is mapped in bot code.
# for example if forward command is required, 'n' is sent to the bot serially and accordingly the bot moves ahead.
command_dict = {

    'forward': 'n',
    'reverse': 's',
    'right-forward': 'r',
    'left-forward': 'l',
    'turn-180': 't',

    'angle_right': 'f',
    'angle_left': 'g',

    'left': 'e',
    'right': 'w',

    'turn-180-left': 'A',
    'turn-180-right': 'D',
    'buzzer-on': ',',
    'buzzer-off': '.',

    'stop': 'e'
}

# ------------------------------------------------------------------- #

initializeArms()

# Remove tree locations from traversable nodes
travesable_nodes.remove(tree_location_1)
travesable_nodes.remove(tree_location_2)
travesable_nodes.remove(tree_location_3)


# Get fruit positions for each tree
fruit_postion_1 = getNeighbouringNodes(tree_location_1)
fruit_postion_2 = getNeighbouringNodes(tree_location_2)
fruit_postion_3 = getNeighbouringNodes(tree_location_3)

print ''

# There are total of 12 bot movements since there are 3 trees and max number of fruits on 1 tree can be 4
# hence total bot movements will be 3 * 4 = 12 which includes picking up and dropping fruits

# Bot has to travel to all fruit locations
fruit_trav_location = fruit_postion_1 + fruit_postion_2 + fruit_postion_3

# bot movements from 1 - 2
# ---------------------------------------------------------------------------------------------------------------------#

# Determine shortest node from w.r.t current position
temp = 1000
shortest_node = 0
for value in fruit_trav_location:
    distance = node_distance(init_start_node, value)
    if distance < temp:
        temp = distance
        shortest_node = value
fruit_trav_location.remove(shortest_node)

# Find Path using a* algorithm
print init_start_node, " --> ", shortest_node
cost, pickup_node, directions, face = a_star_search(init_start_node, shortest_node, travesable_nodes,
                                                    init_direction)

# Check which tree the bot has arrived
if shortest_node in fruit_postion_1:
    dest = tree_location_1
    deposition_node = depostion_zone_apple
elif shortest_node in fruit_postion_2:
    dest = tree_location_2
    deposition_node = depostion_zone_bberry
else:
    dest = tree_location_3
    deposition_node = depostion_zone_orange

# If not facing the tree, make the bot turn
tree_facing_turn, tree_facing_dir = turn_4_tree(face, pickup_node, dest)
if not tree_facing_turn == 'forward':
    directions.append(tree_facing_turn)
face = tree_facing_dir

# Change node to reach one location before destination : This is implemented because it's easier to detect fruits on
#  trees if the bot arrives at the tree from the facing direction. If it arrives from Left or Right, it has to turn
# and might not allign with the tree as expected. Hence if it can arrive from the facing direction compute shortest
#  path w.r.t that node.

shortest_node, changeNode = nodeBeforeShotestNode(shortest_node, face)
# If possible then calculate again
if changeNode:
    print init_start_node, " --> ", shortest_node
    cost, pickup_node, directions, face = a_star_search(init_start_node, shortest_node, travesable_nodes,
                                                        init_direction)
    if shortest_node in fruit_postion_1:
        dest = tree_location_1
        deposition_node = depostion_zone_apple
    elif shortest_node in fruit_postion_2:
        dest = tree_location_2
        deposition_node = depostion_zone_bberry
    else:
        dest = tree_location_3
        deposition_node = depostion_zone_orange

    tree_facing_turn, tree_facing_dir = turn_4_tree(face, pickup_node, dest)
    if not tree_facing_turn == 'forward':
        directions.append(tree_facing_turn)
    face = tree_facing_dir

print "Track 0 :", directions
print 'final facing direction : ', face
print ''

# update the inital direction for next path findings
init_direction = face

# Send commands to Bot
for detected_value in directions:
    if detected_value in command_dict.keys():
        sendCommand(command_dict[detected_value])
        waitForResponse()
if changeNode:
    sendCommand('forward')

# Start Fruit detection
detected_fruit, det_fruit_size = img_detect_4_pi.image_processing()

# Chedk if required Fruit is to be plucked or Not
check_to_pluck = Eliminator(detected_fruit, det_fruit_size)
pluckOrNot(check_to_pluck)
if check_to_pluck:

    if init_direction == 'N':
        init_direction = 'S'
    elif init_direction == 'W':
        init_direction = 'E'
    elif init_direction == 'S':
        init_direction = 'N'
    else:
        init_direction = 'W'

# ---------------------------------------------------------------------------------------------------------------------#

# If fruit Plucked go to DZ or else go to next shortest tree location
# Exectue only if the fruit is plucked or else skip this path and compute path to next fruit location

if check_to_pluck:
    temp_dep = 1000
    shortest_node_dep = 0
    for value in deposition_node:
        distance = node_distance(shortest_node, value)
        if distance < temp_dep:
            temp_dep = distance
            shortest_node_to_dep = value

    print shortest_node, " --> ", shortest_node_to_dep
    cost0, pickup_node0, directions0, face0 = a_star_search(shortest_node, shortest_node_to_dep, travesable_nodes,
                                                            init_direction)

    print 'face0 : ', face0

    init_direction = face0
    print "Track 0 return :", directions0
    print 'final facing direction : ', init_direction
    print ''
    print ''
    for detected_value in directions0:
        if detected_value in command_dict.keys():
            sendCommand(command_dict[detected_value])
            waitForResponse()
    openArms()

# bot movements from 1 - 12
# repeat the same steps as above
# ---------------------------------------------------------------------------------------------------------------------#
for i in range(1, 12):
    if check_to_pluck:
        next_node = shortest_node_to_dep
    else:
        next_node = shortest_node
        print ' skipping DZ :'

    temp = 1000
    shortest_node = 0
    for value in fruit_trav_location:
        distance = node_distance(next_node, value)
        if distance < temp:
            temp = distance
            shortest_node = value

    fruit_trav_location.remove(shortest_node)
    print ''

    print next_node, " --> ", shortest_node
    cost1, pickup_node1, directions1, face1 = a_star_search(next_node, shortest_node,
                                                            travesable_nodes, init_direction)
    if shortest_node in fruit_postion_1:
        dest = tree_location_1
        deposition_node = depostion_zone_apple
    elif shortest_node in fruit_postion_2:
        dest = tree_location_2
        deposition_node = depostion_zone_bberry
    else:
        dest = tree_location_3
        deposition_node = depostion_zone_orange

    tree_facing_turn1, tree_facing_dir1 = turn_4_tree(face1, pickup_node1, dest)
    if not tree_facing_turn1 == 'forward':
        directions1.append(tree_facing_turn1)
    face1 = tree_facing_dir1

    shortest_node, changeNode = nodeBeforeShotestNode(shortest_node, face1)
    if changeNode:
        cost, pickup_node1, directions1, face1 = a_star_search(next_node, shortest_node, travesable_nodes,
                                                               init_direction)
        if shortest_node in fruit_postion_1:
            dest = tree_location_1
            deposition_node = depostion_zone_apple
        elif shortest_node in fruit_postion_2:
            dest = tree_location_2
            deposition_node = depostion_zone_bberry
        else:
            dest = tree_location_3
            deposition_node = depostion_zone_orange

        tree_facing_turn, tree_facing_dir = turn_4_tree(face1, pickup_node1, dest)
        if not tree_facing_turn == 'forward':
            directions1.append(tree_facing_turn)
            face1 = tree_facing_dir

    print "Track ", i, directions1
    print 'final facing direction : ', face1
    print ''
    print ''
    init_direction = face1
    for detected_value in directions1:
        if detected_value in command_dict.keys():
            sendCommand(command_dict[detected_value])
            waitForResponse()
    if changeNode:
        sendCommand('forward')

    detected_fruit, det_fruit_size = img_detect_4_pi.image_processing()
    check_to_pluck = Eliminator(detected_fruit, det_fruit_size)
    pluckOrNot(check_to_pluck)
    if check_to_pluck:

        if init_direction == 'N':
            init_direction = 'S'
        elif init_direction == 'W':
            init_direction = 'E'
        elif init_direction == 'S':
            init_direction = 'N'
        else:
            init_direction = 'W'

    # -----------------------------------------------------------------------------------------------------------------#

    if check_to_pluck:
        temp_dep = 1000
        shortest_node_dep = 0
        for value in deposition_node:
            distance = node_distance(shortest_node, value)
            if distance < temp_dep:
                temp_dep = distance
                shortest_node_to_dep = value

        print shortest_node, " --> ", shortest_node_to_dep
        cost2, pickup_node2, directions2, face2 = a_star_search(shortest_node, shortest_node_to_dep,
                                                                travesable_nodes, init_direction)

        init_direction = face2
        print "Track return ", i, directions2
        print 'final facing direction : ', init_direction
        print ''
        print ''
        for detected_value in directions2:
            if detected_value in command_dict.keys():
                sendCommand(command_dict[detected_value])
                waitForResponse()
        openArms()

    # -----------------------------------------------------------------------------------------------------------------#
    # -----------------------------------------------------------------------------------------------------------------#

# -----------------------------------------------------------------------------------------------------------------#
