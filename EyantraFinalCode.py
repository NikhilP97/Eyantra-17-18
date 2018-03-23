# Libraries
import serial
import time
import RPi.GPIO as GPIO
import img_detect_4_pi # image processing functions imported

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

apple_dict = {'L': 1, 'M': 1, 'S': 1}  # Required Fruits Table
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
    7, 8, 9, 10, 11, 12, 13,
    0, 1, 2, 3, 4, 5, 6,

]

# Stores character values needed to be sent for each command. Same is mapped in bot code.
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

# Servo commands to Open and Close the Arm of the bot used for Plucking & Dropping fruits.
# --------------------------------------------------------#
servo_pwm = 18  # Pin number for the Servo PWM pin number
GPIO.setmode(GPIO.BCM)  # The mode for GPIO pin addressing is set to BCM
GPIO.setup(servo_pwm, GPIO.OUT)  #
p = GPIO.PWM(servo_pwm, 50)
p.start(7.5)


# Arm Open (Servo)
def openArms():
    p.ChangeDutyCycle(6.0)
    time.sleep(1)


# Arm Close (Servo)
def closeArms():
    p.ChangeDutyCycle(3.0)
    time.sleep(1)


openArms()

# --------------------------------------------------------#

'''
*Function Name: Eliminator()
*Input: (fruit,size)
*Output: returns the boolean value 1 if the detected fruit is in the required fruits table, otherwise returns 0
*Logic: This is a function which dynamically stores all the value of fruits which have to be picked. 
        Accordingly, it eliminates the count of the "picked fruits" by updating dictionaries
*Example Call: Eliminator('A','L')
'''


def Eliminator(fruit, size):
    print 'in eliminator'
    if (fruit == 'A'):
        if (apple_dict[size] == 0):
            return 0
        else:
            tempVal = apple_dict[size]
            tempVal = tempVal - 1
            apple_dict[size] = tempVal
            return 1

    elif (fruit == 'B'):
        if (blue_dict[size] == 0):
            return 0
        else:
            tempVal = blue_dict[size]
            tempVal = tempVal - 1
            blue_dict[size] = tempVal
            return 1

    elif (fruit == 'O'):
        if (orange_dict[size] == 0):
            return 0
        else:
            tempVal = orange_dict[size]
            tempVal = tempVal - 1
            orange_dict[size] = tempVal
            return 1
    else:
        return 2


'''
*Function Name: pluckOrNot
*Input: (boolValue)
*Output: decides whether to grip the fruit or not.
         If bool is 1, then it grips the fuit
*Logic: If the Eliminator() function returns that the fruit must be picked,
        then the Rpi sends command to the Firebird to approach the tree.
        Otherwise, another sequence is executed
*Example call: pluckOrNot(1)
'''


def pluckOrNot(boolValue):
    print 'in pluck or not'
    if (boolValue == 1):
        sendCommand('1')
        waitForResponse()
        closeArms()
        sendCommand('d')
        waitForResponse()
    else:
        return


'''
* Function Name: sendCommand()
* Input: (command)
* Output: send the required command to FireBird V using serial communication
* Logic: From the command dictionary if any command is found, then pass it to this
*        function which will then send it to FireBird V.
* Example Call: sendCommand(1)
'''


def sendCommand(command):
    if ser.isOpen():
        print "Sent command: " + command
        ser.write(command)
        ser.reset_output_buffer()
        # for character in command:
        #    ser.write(character)
    else:
        print "Send-Serial not open!"


'''
* Function Name: calculateTurn()
* Input: (current_direction, current_node, next_node)
* Output: Required turn for given change in node, cost for turn, bot direction after turn
* Logic: Based on various possible combinations of turns and faces, output is read
*        from a dictionary
* Example Call: calculateTurn('E', 23, 24)
'''


def calculateTurn(current_dir, current_node, next_node):
    if current_node - next_node == 1:  # e.g. 24 - 23 = 1, movement towards West
        next_dir = 'W'
    elif current_node - next_node == -1:
        next_dir = 'E'
    elif current_node - next_node == -7:
        next_dir = 'N'
    else:
        next_dir = 'S'

    decider = current_dir + next_dir

    options = {  # [turn_required, turn_cost]
        'NN': ['forward', 0],
        'SS': ['forward', 0],
        'EE': ['forward', 0],
        'WW': ['forward', 0],

        'NS': ['turn-180', 0],
        'SN': ['turn-180', 0],
        'EW': ['turn-180', 0],
        'WE': ['turn-180', 0],

        'NW': ['left-forward', turn_cost_90],
        'WS': ['left-forward', turn_cost_90],
        'SE': ['left-forward', turn_cost_90],
        'EN': ['left-forward', turn_cost_90],

        'NE': ['right-forward', turn_cost_90],
        'ES': ['right-forward', turn_cost_90],
        'SW': ['right-forward', turn_cost_90],
        'WN': ['right-forward', turn_cost_90],
    }

    turn_required = options[decider][0]
    turn_cost = options[decider][1]

    # if turn_required == 'reverse':  # For reverse, bot_direction doesnt change.
    #    next_dir = current_dir

    return turn_required, turn_cost, next_dir


# ------------------------------------------------------------------- #

'''
* Function Name: determine_drop_angle() 
* Input: (node, required_dep_zone, last_dir) 
* Output: turning angle ( +45 degrees or - 45 degrees )
* Logic: After reaching the dropping position,
*        the bot has to turn towards the dropping zone to drop the fruit.
*        So this function determines whether the bot should turn in +45 degrees
*        or -45 degrees to drop the fruit.
* Example Call: determine_next_node(4, 20, 'N')
'''


def determine_drop_angle(node, required_dep_zone, last_dir):
    x, y = nodeToCordinate(node)

    if last_dir == 'N':
        check_right = cordinateToNode((x + 1, y))
        if check_right in required_dep_zone:
            print 'im here'
            return 'angle_right'

        else:
            return 'angle_left'

    elif last_dir == 'S':
        check_right = cordinateToNode((x + 1, y))
        if check_right in required_dep_zone:
            print 'im here'
            return 'angle_left'

        else:
            return 'angle_right'
    elif last_dir == 'W':
        check_right = cordinateToNode((x, y + 1))
        if check_right in required_dep_zone:
            print 'im here'
            return 'angle_right'

        else:
            return 'angle_left'
    else:
        check_right = cordinateToNode((x, y + 1))
        if check_right in required_dep_zone:
            print 'im here'
            return 'angle_left'

        else:
            return 'angle_right'


'''
* Function Name: nodeBeforeShotestNode() 
* Input: (currentNode, nodeDirection) 
* Output: destination node, boolean
* Logic: While traversing to the fruit location,
*        the bot has to reach the location from the front and not take a turn.
*        This is so that the fruit kept at any height can come in the field of view of 
*        the camera and detect it properly.
* Example Call: nodeBeforeShotestNode(35, 'N')
'''


def nodeBeforeShotestNode(currentNode, nodeDirection):
    x, y = nodeToCordinate(currentNode)
    node = 0
    if nodeDirection == 'N':
        if y - 1 > 0:
            node = cordinateToNode((x, y - 1))
            if node in travesable_nodes:
                return node, True
        else:
            return node, False
    elif nodeDirection == 'W':
        if x + 1 < 7:
            node = cordinateToNode((x + 1, y))
            if node in travesable_nodes:
                return node, True
        else:
            return node, False
    elif nodeDirection == 'E':
        if x - 1 > 0:
            node = cordinateToNode((x - 1, y))
            if node in travesable_nodes:
                return node, True
        else:
            return node, False
    else:
        if y + 1 < 7:
            node = cordinateToNode((x, y + 1))
            if node in travesable_nodes:
                return node, True
        else:
            return node, False


# ---------------------------------------------------------------------------------------------------------------------#
'''
* Function Name: determine_next_node() 
* Input: (node, required_dep_zone, last_dir) 
* Output: next node
* Logic: After reaching the dropping position,
*        the bot has to turn towards the dropping zone to drop the fruit.
*        So this function determines whether the bot should make the required
*        turn and in which direction.
* Example Call: determine_next_node(20, 30, 'S')
'''


def determine_next_node(node, required_dep_zone, last_dir):
    x, y = nodeToCordinate(node)

    if last_dir == 'N' or last_dir == 'S':
        check_right = cordinateToNode((x + 1, y))
        if check_right in required_dep_zone:
            print 'im here'
            return cordinateToNode((x + 1, y))

        else:
            return cordinateToNode((x - 1, y))
    else:
        check_right = cordinateToNode((x, y + 1))
        if check_right in required_dep_zone:
            print 'im here'
            return cordinateToNode((x, y + 1))

        else:
            return cordinateToNode((x, y - 1))


'''
* Function Name: turn_4_dropping_fruit() 
* Input: (dir, cur_node, dep_zone) 
* Output: drop_turn_required, dir_4_drop
* Logic: After reaching the dropping position,
*        the bot has to turn towards the dropping zone to drop the fruit.
*        So this function determines the required turn to face the dropping zone.
*        It also updates the final direction which the bot will face.
* Example Call: turn_4_dropping_fruit('W', 11, 12)
'''


def turn_4_dropping_fruit(dir_last, cur_node, dep_zone):
    nex_node = determine_next_node(cur_node, dep_zone, dir_last)
    print 'next node : ', nex_node
    print 'dir_last : ', dir_last
    print 'current node : ', cur_node
    drop_turn_required, drop_turn_cost, dir_after_drop = calculateTurn(dir_last, cur_node, nex_node)
    if drop_turn_required == 'left-forward':
        drop_turn_required = 'left'
    elif drop_turn_required == 'right-forward':
        drop_turn_required = 'right'

    return drop_turn_required, dir_after_drop


'''
* Function Name: turn_4_tree() 
* Input: (dir, cur_node, nex_node) 
* Output: fruit_turn_required, dir_4_fruit
* Logic: After reaching the fruit position,
*        the bot has to turn towards the tree to identify the fruit.
*        So this function determines the required turn to face the fruit.
*        It also updates the final direction which the bot will face.
* Example Call: turn_4_tree('E', 29, 36)
'''


def turn_4_tree(dir_last, cur_node, nex_node):
    fruit_turn_required, fruit_turn_cost, dir_4_fruit = calculateTurn(dir_last, cur_node, nex_node)
    if fruit_turn_required == 'reverse':  # If its reverse at the end of path, turn 180 degrees
        fruit_turn_required = 'turn-180'
        if dir_4_fruit == 'N':
            dir_4_fruit = 'S'
        elif dir_4_fruit == 'W':
            dir_4_fruit = 'E'
        elif dir_4_fruit == 'S':
            dir_4_fruit = 'N'
        else:
            dir_4_fruit = 'W'
    # Remove the extra 'forward' at the end of path
    elif fruit_turn_required == 'left-forward':
        fruit_turn_required = 'left'
    elif fruit_turn_required == 'right-forward':
        fruit_turn_required = 'right'
    # elif fruit_turn_required == 'forward':
    #     fruit_turn_required.pop()

    return fruit_turn_required, dir_4_fruit


# ------------------------------------------------------------------- #


'''
* Function Name: cordinateToNode() --> Co-ordinate to Node Number
* Input: (cord_tuple) --> cord tuple to be converted to Node Number
* Output: The Node Number for the Cell as an integer
* Logic: Node Number is easier to work with in some cases,
*        whereas, Cords are easier in some. So they can
*        be easily interchanged.
*        Subtract 1 and combine the x and y parts
* Example Call: cordinateToNode((3, 4))
'''


def cordinateToNode(cord_tuple):
    x, y = cord_tuple
    return y * 7 + x


# ------------------------------------------------------------------- #


'''
* Function Name: nodeToCordinate() --> Node Number to Co-ordinate
* Input: (node_num) --> Node Number to be converted
* Output: The Cell Number(Co-ordinates) as a tuple
* Logic: Node Number is easier to work with in some cases,
*        whereas, Cords are easier in some. So they can
*        be easily interchanged.
*        Separate the digits and add 1 to each.
* Example Call: nodeToCordinate(43)
'''


def nodeToCordinate(node_num):
    # print 'node_num is : ', node_num
    # y = ((node_num + 1) / 7) - 1
    var = node_num / 7
    multiplier = var * 7
    divider = multiplier / 7
    y = divider
    x = node_num - multiplier
    # x = (node_num + 1) / 7

    return x, y


# ------------------------------------------------------------------- #


'''
* Function Name: node_distance()
* Input: (node1, node2) --> the 2 Node Numbers
* Output: The distance between the two nodes, which is the number of
*         steps/jumps from node1 to node2 as an integer
* Logic: Convert to cords. Distance is sum of steps taken in x-axis and
*        steps take on y-axis.
* Example Call: node_distance(72, 37)
'''


def node_distance(node1, node2):
    a1, b1 = nodeToCordinate(node1)
    a2, b2 = nodeToCordinate(node2)

    return abs(a1 - a2) + abs(b1 - b2)


# ------------------------------------------------------------------- #


'''
* Function Name: getNeighbouringNodes() --> Finds the adjacent nodes for a passed node
* Input: (node) --> The Node Number whose neighbours have to be found out.
* Output: Node numbers of adjacent cells as a list (Even if not traversable)
* Logic: Convert node number to cords, separate as x and y.
*        Adjacent cells are simply obtained by adding/subtracting 1
*        to the x and y parts, one at a time, if the result is a valid
*        cell number.
* Example Call: getNeighbouringNodes(82)
'''


def getNeighbouringNodes(node):
    x, y = nodeToCordinate(node)
    # print 'x, y position : ',x,y
    neighbours = []

    if y - 1 >= 0:  # If exists
        neighbours.append(cordinateToNode((x, y - 1)))
    if x - 1 >= 0:
        neighbours.append(cordinateToNode((x - 1, y)))
    if y + 1 < 7:
        neighbours.append(cordinateToNode((x, y + 1)))
    if x + 1 < 7:
        neighbours.append(cordinateToNode((x + 1, y)))

    return neighbours


# ------------------------------------------------------------------- #


'''
* Function Name: waitForResponse()
* Input: None
* Output: (response received from bot as char)
* Logic: If serial port is open(which was initialized at the start), wait till a character
*        arrives
* Example Call: waitForResponse()
'''


def waitForResponse():
    # received = None
    print ' awaiting response'
    if ser.isOpen():

        while ser.inWaiting() == 0:
            continue
        size = ser.inWaiting()
        received = ser.read(1)
        print 'size :', size
        if size:

            print "Received: " + received
        else:
            print 'invalid character received'
        time.sleep(0.5)
    else:
        print "Read-Serial not open!"

    ser.reset_input_buffer()
    print ''


'''
* Function Name: a_star_search() --> Performs A* path finding for the passed nodes
* Input: (startNode, goalNode, traversable_nodes, current_dir) -->
*         Start of the path, Goal of the path, List of passable(traversable) nodes and current facing direction
* Output: -1 if no path is found, else: path cost, one node before goal(for pickup), path, final facing direction
* Logic: The standard A* algorithm is modified in which g_cost is determined from the node_to_node and turn costs, and
*        directions are stored along with path for the next loop run
* Example Call: a_star_search(33, 17, traversable_nodes, 'N')
'''


def a_star_search(startNode, goalNode, traversable_nodes, current_dir):
    openNodes = []
    closedNodes = []

    traversable_nodes.append(startNode)
    traversable_nodes.append(goalNode)
    pathList = []  # Final output list the path

    nodeParents = {}  # Stores node : parent(node from which it comes)
    fCost = {}  # Stores node : fCost
    gCost = {}  # Stores node : gCost
    nodeDirections = {}  # Direction at every node
    botDirections = []  # Directional commands for bot

    # initial fCost = gCost + hCost #
    fCost[startNode] = 0 + node_distance(startNode, goalNode) * cell_to_cell_cost
    gCost[startNode] = 0

    # Stores turn direction required at each node
    nodeDirections[startNode] = current_dir

    openNodes.append(startNode)

    while 1:
        fCostCompare = 9999  # large value to find minimum
        currentNode = 0

        if len(openNodes) == 0:  # If object not reachable
            traversable_nodes.remove(startNode)  # rectify the modified list
            traversable_nodes.remove(goalNode)
            return -1

        for node in openNodes:  # Set node with least fCost as current
            if fCost[node] < fCostCompare:
                currentNode = node
                fCostCompare = fCost[node]

        openNodes.remove(currentNode)  # Move from open to closed
        closedNodes.append(currentNode)

        if currentNode == goalNode:  # If reached goal
            break

        neighbours = getNeighbouringNodes(currentNode)

        # For every neighbour get fCost and append to open nodes if valid--- #
        for j in neighbours:
            if j not in traversable_nodes or j in closedNodes:
                continue

            # get the turnRequired, costs and directions to get from currentNode to neighbour
            turnRequired, turnCost, nextDir = calculateTurn(nodeDirections[currentNode][0], currentNode, j)
            tentativeGCost = gCost[currentNode] + turnCost + node_distance(currentNode, j) * cell_to_cell_cost

            if j not in openNodes:
                openNodes.append(j)
            elif tentativeGCost >= gCost[j]:  # skip this
                continue

            # Store costs and update parent, directions
            gCost[j] = tentativeGCost
            fCost[j] = gCost[j] + node_distance(j, goalNode) * cell_to_cell_cost
            nodeParents[j] = currentNode
            nodeDirections[j] = [nextDir, turnRequired]

    pathNode = goalNode
    pathList.append(goalNode)

    # Form path list by retracing path
    while pathNode != startNode:
        pathNode = nodeParents[pathNode]
        pathList.append(pathNode)

    pathList.reverse()
    print 'final path list : ', pathList
    # create a directions_list for bot commands
    for j in range(0, len(pathList) - 1):
        contents = nodeDirections[pathList[j + 1]]
        botDirections.append(contents[1])

    pickup_node_in_astar = pathList[-1]
    final_facing_dir = nodeDirections[goalNode][0]  # bot_facing_direction at the end of path

    traversable_nodes.remove(startNode)  # rectify the modified list
    traversable_nodes.remove(goalNode)

    return fCost[goalNode], pickup_node_in_astar, botDirections, final_facing_dir


# ------------------------------------------------------------------- #

# Remove tree locations from traversable nodes
travesable_nodes.remove(tree_location_1)
travesable_nodes.remove(tree_location_2)
travesable_nodes.remove(tree_location_3)


# Get fruit positions for each tree
fruit_postion_1 = getNeighbouringNodes(tree_location_1)
fruit_postion_2 = getNeighbouringNodes(tree_location_2)
fruit_postion_3 = getNeighbouringNodes(tree_location_3)

print ''
# ---------------------------------------------------------------------------------------------------------------------#

# Bot has to travel to all fruit locations
fruit_trav_location = fruit_postion_1 + fruit_postion_2 + fruit_postion_3

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

# Change node to reach one location before destination
shortest_node, changeNode = nodeBeforeShotestNode(shortest_node, face)
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
print ''

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

# ---------------------------------------------------------------------------------------------------------------------#
for i in range(1, 4):
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

for i in range(0, 4):

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
    cost3, pickup_node3, directions3, face3 = a_star_search(next_node, shortest_node,
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

    tree_facing_turn3, tree_facing_dir3 = turn_4_tree(face3, pickup_node3, dest)
    if not tree_facing_turn3 == 'forward':
        directions3.append(tree_facing_turn3)
    face3 = tree_facing_dir3

    shortest_node, changeNode = nodeBeforeShotestNode(shortest_node, face3)
    if changeNode:
        print next_node, " --> ", shortest_node
        cost, pickup_node3, directions3, face3 = a_star_search(next_node, shortest_node, travesable_nodes,
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

        tree_facing_turn, tree_facing_dir = turn_4_tree(face3, pickup_node3, dest)
        if not tree_facing_turn == 'forward':
            directions3.append(tree_facing_turn)
            face3 = tree_facing_dir

    print "Track second fruit ", i, directions3
    print 'final facing direction : ', face3
    print ''
    print ''
    init_direction = face3
    for detected_value in directions3:
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
        cost4, pickup_node4, directions4, face4 = a_star_search(shortest_node, shortest_node_to_dep,
                                                                travesable_nodes, init_direction)

        init_direction = face4
        print "Track  return ", i, directions4
        print 'final facing direction : ', init_direction
        print ''
        print ''
        for detected_value in directions4:
            if detected_value in command_dict.keys():
                sendCommand(command_dict[detected_value])
                waitForResponse()
        openArms()

    # -----------------------------------------------------------------------------------------------------------------#

for j in range(0, 4):

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

    if len(fruit_trav_location) > 0:
        fruit_trav_location.remove(shortest_node)
    print ''

    print next_node, " --> ", shortest_node
    cost5, pickup_node5, directions5, face5 = a_star_search(next_node, shortest_node,
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

    tree_facing_turn5, tree_facing_dir5 = turn_4_tree(face5, pickup_node5, dest)
    if not tree_facing_turn5 == 'forward':
        directions5.append(tree_facing_turn5)
    face5 = tree_facing_dir5

    shortest_node, changeNode = nodeBeforeShotestNode(shortest_node, face5)
    if changeNode:
        print next_node, " --> ", shortest_node
        cost, pickup_node5, directions5, face5 = a_star_search(next_node, shortest_node, travesable_nodes,
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

        tree_facing_turn, tree_facing_dir = turn_4_tree(face5, pickup_node5, dest)
        if not tree_facing_turn == 'forward':
            directions5.append(tree_facing_turn)
            face5 = tree_facing_dir

    print "Track second fruit ", i, directions5
    print 'final facing direction : ', face5
    print ''
    print ''
    init_direction = face5
    for detected_value in directions5:
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

        print 'shortest node to depositon zone is :', shortest_node_to_dep
        print shortest_node, " --> ", shortest_node_to_dep
        cost6, pickup_node6, directions6, face6 = a_star_search(shortest_node, shortest_node_to_dep,
                                                                travesable_nodes, init_direction)

        init_direction = face6
        print "Track  return ", i, directions6
        print 'final facing direction : ', init_direction
        print ''
        print ''
        for detected_value in directions6:
            if detected_value in command_dict.keys():
                sendCommand(command_dict[detected_value])
                waitForResponse()
        openArms()

    # -----------------------------------------------------------------------------------------------------------------#
    # -----------------------------------------------------------------------------------------------------------------#

    # -----------------------------------------------------------------------------------------------------------------#
