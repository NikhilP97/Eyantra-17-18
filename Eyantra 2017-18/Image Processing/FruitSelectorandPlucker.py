import serialCommands


apple_dict = {'L': 1, 'M': 1, 'S': 1}  # Required Fruits Table
blue_dict = {'L': 1, 'M': 1, 'S': 1}
orange_dict = {'L': 0, 'M': 2, 'S': 0}


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

#----------------------------------------------------------------------------------------------------------------------#


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

#----------------------------------------------------------------------------------------------------------------------#