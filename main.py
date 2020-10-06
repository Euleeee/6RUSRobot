from sixRUS import sixRUS
import time
import controller as con
import RPi.GPIO as GPIO


def circle(dir=0):
    import numpy as np
    import math as m

    n = 1
    r = 35
    resolution = 20

    t = np.linspace(0, n*2*m.pi, resolution*n)

    for num in t:
        if dir == 0:
            x = m.cos(num)*r
            y = m.sin(num)*r
        else:
            x = m.cos(num)*r
            y = m.sin(num-m.pi)*r

        robo.mov([x, y, -100, 0, 0, 0])

# circle()


# Main program
def main():
    mode = 'demo'  # current mode (check documentation for all possible modes)
    controllerConnected = False  # flag if controller is connected
    global ss

    robo = sixRUS(stepperMode=1/32, stepDelay=0.002)

    robo.homing('90')

    joy = con.initCont()

    if joy is not None:  # controller connected => manual mode
        mode = 'manual'

    while True:  # infinite loop
        pass


while True:
    time.sleep(0.001)
    ans = con.listen2Cont(joy, robo.currPose)

    if isinstance(ans, str):  # unexpected string
        break
    else:
        print(ans)
        robo.mov(ans)

print('End')


# Main program if this file get executed
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  # shutdown python program gently
        GPIO.cleanup()
    print("6-RUS program was terminated by Keyboard-Interrupt. Please start the program again to control the robot again!")
