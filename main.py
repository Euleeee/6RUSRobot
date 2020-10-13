from sixRUS import sixRUS
import time
import controller as con
import RPi.GPIO as GPIO
import demo 
import types
import random

from threading import Timer, Event

# GLOBALS
programStopped = Event()  # Event to set if program gets
shouldNotListen2Cont = Event()  # Event for checking if the program should listen to the Controller
robotMode = ''
joystick = None  # global varriable for joystick-class
alreadyConnected = False  # check if contoller reconnected

def call_every_5_sec():
    """execute everything in here every 5 seconds after"""
    if not programStopped.is_set():  # only execute routine if program is not terminated

        # TODO. comment a bit more
        connected = con.stillConnected()  # check if controller is connected
        
        global shouldNotListen2Cont
        global alreadyConnected
        
        if connected == False:  # not connected
            alreadyConnected = False
            print("Please connect controller! Retrying in 5 seconds...")
            
        else:  # connected

            if alreadyConnected:  # controller is still connected
                print('Controller still connected.')
                # no new initialisation required here
            else:
                stopListening2Cont()  # stop listening as the controller gets initalised 
                init_global_joystick()  # init new joystick since the controller was reconnected or connected the first time
                startListening2Cont()
                alreadyConnected = True
                print('Controller connected.')

        Timer(5.0, call_every_5_sec).start()

def call_every_tenth_sec():
    # TODO: add discription
    if not programStopped.is_set() and not shouldNotListen2Cont.is_set():  # only execute routine if program is not terminated

        global joystick
        ans = con.listen2Cont(joystick)

        eval_controller_response(ans)  # evaluate the answer from controller

        Timer(0.1, call_every_tenth_sec).start()

def init_global_joystick():
    # TODO: discription

    global joystick
    joystick = con.initCont()
    return joystick


def eval_controller_response(response):
    """evaluates the answer from the listen2Cont-function"""
    if isinstance(response, str):
        # controller gave an answer

        global robotMode

        if response == 'stop':
            pass
        elif response == 'homing':
            stopListening2Cont()
            pass
        elif response == 'demo':
            pass
        elif response == 'manual':
            pass
        elif response == 'calibrate' :
            pass
        else:
            raise Exception("Unknown answer from controller")
        
        if robotMode != response:  # only print if the mode changes
            print('Switching to:', response)

        robotMode = response  # set robot mode to the response
    else:
        # controller has given a pose
        pass
        

def startListening2Cont():
    """start listening to controller every 0.1 s"""
    global shouldNotListen2Cont
    shouldNotListen2Cont.clear()
    Timer(0.1, call_every_tenth_sec).start()

def stopListening2Cont():
    """stop listening to controller. Needed if a program is listening to 
    the controller itself or a new joystick gets initialised"""
    global shouldNotListen2Cont
    shouldNotListen2Cont.set()


def mov_with_controller(robot, dt=0.001):
    """""" # TODO: enter discription

    ans = ''
    global joystick

    while True:  # infinite loop
        time.sleep(dt)
        ans = con.listen2Cont(joystick, robot.currPose)

        if isinstance(ans, str):  # string was given as an answer
            eval_controller_response(ans)
            
            break  # break out of infinite loop

        else:  # pose was given as an answer
            robot.mov(ans)  # move robot (PTP)


def move_with_demo(robot):
    """Selects a random demo programm and executes it"""

    modules = list_of_modules(demo)
    prog = random.choice(modules)  # choose a random demo
    demoPosList = prog()  # execute chosen demo programm

    global robotMode

    for pos in demoPosList:
        try:
            if pos[6] == 'lin':
                coord = pos[:6]  # extract only pose
                robot.mov_lin(coord)  # move linear
            elif pos[6] == 'mov':
                coord = pos[:6]  # extract only pose
                robot.mov(coord)  # move with PTP-interplation
        except IndexError:
            robot.mov(pos)  # if 'lin' or 'mov' wasent given, use mov/PTP
        
        if not robotMode == 'demo':  # break if the mode was changed
            break
        

def list_of_modules(packageName):
    """Find all modules in a package 
    `packageName`: Name of package
    `return`: List of all modules in this package
    """
    modulList = []
    for a in dir(demo):
        if isinstance(getattr(demo, a), types.FunctionType):
            modulList.append(getattr(demo,a))

    return modulList

def calibrate_process(robot, dt=0.001):
    """""" # TODO: enter discription

    ans = ''
    global joystick
    motNum = 0  # Motor numbers from 0 to 5
    while True:  # infinite loop
        time.sleep(dt)
        ans, motNum = con.listen2Cont(joystick, robot.currPose, 'calibrate', motNum) 

        if isinstance(ans, str):  # string was given as an answer
            eval_controller_response(ans)
            
            break  # break out of infinite loop

        else:  # pose was given as an answer
            robot.mov_steps(ans)  # move robot (PTP)

############## Main function ###################
def main():
    global robotMode
    robotMode = 'demo'  # current mode (check documentation for all possible modes)

    robo = sixRUS(stepperMode=1/32, stepDelay=0.002)  # init robot

    robo.homing('90')  # home robot

    init_global_joystick()

    call_every_5_sec()  # call subroutine every n-seconds TODO: manage thread
    startListening2Cont()  # start listening to controller
    
        
    while True:  # infinite loop only breaks on Keyboard-Interrupt
        # time.sleep(0.5)
        while robotMode == 'demo':  # TODO
            move_with_demo(robo)
            time.sleep(2)  # wait and then execute the next function
            
        while robotMode == 'homing':
            stopListening2Cont()  # stop listening to controller to prevent program change while homing
            time.sleep(1.5)  # wait a bit to reduce multiple homing attempts
            robo.homing('90')  # use homing method '90'
            startListening2Cont()  # listen again
            robotMode = 'stop'  # exit homing

        while robotMode == 'manual':  # controll the robot with the controller
            stopListening2Cont()  # stop listening to controller (bc. we listen all the time in here)
            mov_with_controller(robo)
            startListening2Cont()  # let the program listen to the controller periodically again

        while robotMode == 'stop':  # stop robot after next movement and do nothing
            firstTime = True
            while robotMode == 'stop':
                if firstTime is True:
                    print("Stopped robot!")
                    firstTime = False
                time.sleep(0.0001)  # limit loop time

        while robotMode == 'calibrate':
            stopListening2Cont()  # stop listening to controller (bc. we listen all the time in here)
            calibrate_process(robo)  
            startListening2Cont()  # let the program listen to the controller periodically again
            robotMode('stop')



# Main program if this file get executed
if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:  # shutdown python program gently
        print('Stopped with KeyboardInterrupt!')
    finally:
        GPIO.cleanup()  # cleanup GPIOs (to avoid warning on next startup)
        programStopped.set()  # set event for stopping threading
        # Exiting message
        print("\n6-RUS program was terminated due to user-input or an error (Please wait ca. 5s) \nPlease start the program again to control the robot again!")
