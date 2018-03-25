import aStarHelperFunctions

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