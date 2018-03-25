# Servo commands to Open and Close the Arm of the bot used for Plucking & Dropping fruits.

import RPi.GPIO as GPIO

# --------------------------------------------------------#

def initializeArms():

    servo_pwm = 18  # Pin number for the Servo PWM pin number
    GPIO.setmode(GPIO.BCM)  # The mode for GPIO pin addressing is set to BCM
    GPIO.setup(servo_pwm, GPIO.OUT)  #
    p = GPIO.PWM(servo_pwm, 50)
    p.start(7.5)
    openArms()

# Arm Open (Servo)
def openArms():
    p.ChangeDutyCycle(6.0)
    time.sleep(1)


# Arm Close (Servo)
def closeArms():
    p.ChangeDutyCycle(3.0)
    time.sleep(1)


# --------------------------------------------------------#