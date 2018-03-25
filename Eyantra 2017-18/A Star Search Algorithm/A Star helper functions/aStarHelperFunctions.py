


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

