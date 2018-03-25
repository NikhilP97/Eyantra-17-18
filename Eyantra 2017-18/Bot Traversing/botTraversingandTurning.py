

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

#---------------------------------------------------------------------------------------------------------------------#

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

#---------------------------------------------------------------------------------------------------------------------#

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

#---------------------------------------------------------------------------------------------------------------------#

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