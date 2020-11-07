from time import sleep
import RPi.GPIO as GPIO

def doSteps(stepPin:int, dirPin:int, direction=1, nrOfSteps:int=1, delay:float=0.02):
    """
    Makes desired steps in one motor (Pins have to be initialized already)
    `stepPin`: int  Step GPIO Pin
    `dirPin`: int  Direction GPIO Pin

    optional:
    `direction`:numeric direction > 0 -> positive, direction <= 0 -> negative
    `nrOfSteps`: int  number of steps to turn  
    `delay`: float  delay between steps in [s]
    """
    # normalize direction to 0 or 1
    if direction > 0:
        direction = 1
    else:
        direction = 0

    GPIO.output(dirPin, direction)  # set direction

    for i in range (nrOfSteps):
        GPIO.output(stepPin, GPIO.HIGH)
        sleep(delay)
        GPIO.output(stepPin, GPIO.LOW)
        sleep(delay)


def doMultiStep(pins2step:list, stepPins:list, dirPins:list, directions:list=[1,1,1,1,1,1], delay:float=0.02):
    """
    Makes one (or zero) step(s) for each motor simultaniously. All list have to be the same length
    (Pins have to be initialized already)

    `pins2Step`: list of boolean (0 or 1)   1 := do a step   0 := do no step
    `stepPins`: list of int   corresponding GPIO Pin numbers to make a step on
    `dirPins`: list of int    corresponding Direction GPIO Pin numbers
    `directions`: list of boolean (0 o 1)    corresponding direction
    `delay`: float  delay between steps in [s]
    """
    # normalize direction to 0 or 1
    for dir in directions:
        if dir > 0: dir = 1        
        else: dir = 0

    # set all direction pins
    for i, dir in enumerate(directions):
        GPIO.output(dirPins[i], dir)

    # set step-pins high
    for i, takeStep in enumerate(pins2step):
        if takeStep > 0: GPIO.output(stepPins[i], GPIO.HIGH)

    sleep(delay)
    
    # set step-pins low again
    for i, takeStep in enumerate(pins2step):
        if takeStep > 0: GPIO.output(stepPins[i], GPIO.LOW)

    sleep(delay)
    