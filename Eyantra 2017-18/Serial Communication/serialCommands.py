import serial

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

#---------------------------------------------------------------------------------------------------------------------#


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