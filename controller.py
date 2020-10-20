import pygame
import time
import os
from math import radians, degrees

def initCont():
    """Inits controller to use it and returns joystick class.
    `returns` `None` if controller is not connected"""
    pygame.joystick.quit()
    pygame.init()
    pygame.joystick.init()

    try:  # check if controller is connected
        joystick = pygame.joystick.Joystick(0)  # assign the controller as joystick
        joystick.init()
    except Exception:  # not connected
        return None

    return joystick

def stillConnected():
    """Checks if a Controller is still connected trough a linux command.
    `returns` boolean"""

    print('Checking connection to controller:')
    controllerStatus = os.system('ls /dev/input/js0')  # checking for controller with linux

    if controllerStatus != 0:  # not connected
        print("Please connect controller! Retrying in 5 seconds...")
        return False
    else:
        return True

def get_movement_from_cont(controls, currentPose):
    """Calculates new pose from controller-input ans returns it as a list
    `controls`:dict  inputs from controller
    `currentPose`:list  poselist of current pose"""

    pose = currentPose
    pose = [pose[0], pose[1], pose[2], degrees(pose[3]), degrees(pose[4]), degrees(pose[5])]  # convert angles to DEG

    # 0Z---> y
    # |
    # V x 

    # speedfactors
    rotFac = 0.1  # Rotationspeed
    transFac = 1  # Translationspeed

    # add controller inputs to values
    pose[0] += controls['LS_UD'] * transFac
    pose[1] += controls['LS_LR'] * transFac
    pose[2] += controls['RS_UD'] * transFac * -1
    pose[3] += controls['LEFT']  * rotFac
    pose[3] -= controls['RIGHT'] * rotFac
    pose[4] += controls['DOWN']  * rotFac
    pose[4] -= controls['UP']    * rotFac
    pose[5] += controls['L1']    * rotFac
    pose[5] -= controls['R1']    * rotFac

    # move pose within workingspace if its outside
    pose = checkMaxVal(pose, 40, 40, [-140, -60], 40, 40, 30)  # this uses degrees

    pose = [pose[0], pose[1], pose[2], radians(pose[3]), radians(pose[4]), radians(pose[5])]  # convert to RAD

    return pose

def get_controller_inputs(joystick):
    """Gets all inputs from controller and returns them as a dict"""
 
    pygame.event.get()  # get event
    pygame.event.clear()  # clear events in queue (only one event needed)

    input = {
        # buttons:
        'xBut': joystick.get_button(0),
        'oBut': joystick.get_button(1),
        'triangBut': joystick.get_button(2),
        'squareBut': joystick.get_button(3),
        # start/select/PS:
        'SELECT': joystick.get_button(8),
        'START': joystick.get_button(9),        
        'PS': joystick.get_button(10),
        # control pad:
        'UP': joystick.get_button(13),
        'DOWN': joystick.get_button(14),
        'LEFT': joystick.get_button(15),
        'RIGHT': joystick.get_button(16),
        # trigger:
        'L1': joystick.get_button(4),
        'R1': joystick.get_button(5),
        'L2': joystick.get_button(6),
        'R2': joystick.get_button(7),
        'L2_': joystick.get_axis(2),  # as axis (no boolean)
        'R2_': joystick.get_axis(5),
        # joysticks:
        'LS_LR': joystick.get_axis(0),  # LS = left stick
        'LS_UD': joystick.get_axis(1),
        'LS': joystick.get_button(11), 
        'RS_LR': joystick.get_axis(3),  # RS = right stick
        'RS_UD': joystick.get_axis(4),
        'RS': joystick.get_button(12),
    }
    return input

def mode_from_inputs(inputs):

    # Buttons on the right side
    xBut = inputs['xBut']
    oBut = inputs['xBut']
    triangBut = inputs['triangBut']
    squareBut = inputs['squareBut']

    # START and SELECT
    startBut = inputs['START']
    selectBut = inputs['SELECT']

    # modes and returning of the mode as string
    if oBut == 1 and xBut == 0 and triangBut == 0 and squareBut == 0:
        return 'stop'
    elif xBut == 1 and triangBut == 1 and squareBut == 1 and oBut == 0:  # do homing procedure
        return 'homing'
    elif startBut == 0 and selectBut == 1:  # start demo program
        return 'demo'
    elif startBut == 1 and selectBut == 0:  # change to manual control with controller
        return 'manual'
    elif xBut == 1 and triangBut == 1 and squareBut == 0 and oBut == 1:  # calibrate motors
        return 'calibrate' 

    return None

def listen2Cont(joystick, currPose=[0,0,0,0,0,0], contMode = '', currMotorNum = 0):
    """listens to controller once and returns pose
    `joystick`: Joystick class (given in init function)
    `currPose`: current pose of the robot
    `contMode`: Controller Mode -> for calibration set contMode = 'calibration', else do nothing
    `cuuMotorNum`: Number between 0 and 5, is the current motor the calibrate
    `return`: pose-list with altered values (if return is a string it is a code or an error)
    """
    if joystick is None:  # if no controller was given
        return 'demo'  # return to demo mode

    pos = currPose  # make own varriable to write to

    pos = [pos[0], pos[1], pos[2], degrees(pos[3]), degrees(pos[4]), degrees(pos[5])]  # convert angles to DEG

    # preinitialize increment-values
    dposx, dposy, dposz = 0, 0, 0
    dposaneg, dposapos, dposbneg, dposbpos, dposcneg, dposcpos = 0, 0, 0, 0, 0, 0

    pygame.event.get()  # get event

    # Buttons on the right side
    xBut = joystick.get_button(0)  # x-Button
    oBut = joystick.get_button(1)  # circle-Button
    triangBut = joystick.get_button(2)  # triangle-Button
    squareBut = joystick.get_button(3)  # square-Button

    # START ans SELECT
    startBut = joystick.get_button(9)  # start
    selectBut = joystick.get_button(8)  # select
        
    # modes and returning of the mode as string
    if oBut == 1 and xBut == 0 and triangBut == 0 and squareBut == 0:
        return 'stop'
    elif xBut == 1 and triangBut == 1 and squareBut == 1 and oBut == 0:  # do homing procedure
        return 'homing'
    elif startBut == 0 and selectBut == 1:  # start demo program
        return 'demo'
    elif startBut == 1 and selectBut == 0:  # change to manual control with controller
        return 'manual'
    elif xBut == 1 and triangBut == 1 and squareBut == 0 and oBut == 1:  # calibrate
        return 'calibrate' 

    if contMode == 'calibration':

        caliMot = [0,0,0,0,0,0]
        if joystick.get_button(4):  # check if L1 was pressed
            currMotorNum -= 1  # select motor for calibration 
        elif joystick.get_button(5):  # check if R1 was pressed
            currMotorNum += 1  # select motor for calibration 

        if currMotorNum > 5: # check if selected motor number exists
            currMotorNum = 0
        elif currMotorNum < 0: 
            currMotorNum = 5    

        if joystick.get_button(13): 
            caliMot[currMotorNum] = 1  # set 1 posivitve for selected motor
        elif joystick.get_button(14): 
            caliMot[currMotorNum] = -1  # set -1 posivitve for selected motor
        else:
            caliMot = [0,0,0,0,0,0]
        
        pygame.event.clear()  # clear events in queue (only one event needed)

        return caliMot, currMotorNum

    else:
        
        dposx = joystick.get_axis(1)        # x-Achse(Linker Joystick unten-> positiv, oben -> negativ)         0Z---> y
        dposy = joystick.get_axis(0)        # y-Achse(Linker Joystick rechts -> positiv, links -> negativ)      |
        dposz = joystick.get_axis(4) *-1    # z-Achse(Rechter Joystick oben -> positiv, unten -> negativ)       \/ x 
        dposaneg = joystick.get_button(15)
        dposapos = joystick.get_button(16)
        dposbneg = joystick.get_button(14)
        dposbpos = joystick.get_button(13)
        dposcneg = joystick.get_button(4)
        dposcpos = joystick.get_button(5)

        pygame.event.clear()  # clear events in queue (only one event needed)

        # add increments to values
        pos[0] += dposx
        pos[1] += dposy
        pos[2] += dposz
        pos[3] += dposaneg
        pos[3] -= dposapos
        pos[4] += dposbneg
        pos[4] -= dposbpos
        pos[5] += dposcneg
        pos[5] -= dposcpos

        pos = checkMaxVal(pos, 40, 40, [-140, -60], 40, 40, 30)  # this uses degrees

        pos = [pos[0], pos[1], pos[2], radians(pos[3]), radians(pos[4]), radians(pos[5])]  # convert to RAD
        
        return pos
    


def checkMaxVal(val,maxX,maxY,zBounds,maxA,maxB,maxC):
        # Maximale x-Richtung
    if val[0] > maxX:
        val[0] = maxX
    if val[0] < -maxX:
        val[0] = -maxX

        # Maximale y-Richtung
    if val[1] > maxY:
        val[1] = maxY
    if val[1] < -maxY:
        val[1] = -maxY

        # Maximale z-Richtung
    if val[2] > zBounds[1]:
        val[2] = zBounds[1]
    if val[2] < zBounds[0]:
        val[2] = zBounds[0]

        # Maximale a-Richtung
    if val[3] > maxA:
        val[3] = maxA
    if val[3] < -maxA:
        val[3] = -maxA

        # Maximale b-Richtung
    if val[4] > maxB:
        val[4] = maxB
    if val[4] < -maxB:
        val[4] = -maxB

        # Maximale c-Richtung
    if val[5] > maxC:
        val[5] = maxC
    if val[5] < -maxC:
        val[5] = -maxC

    return val


# Example on how to use it in the main function
if __name__ == '__main__':
    joy = initCont()

    pose = [0, 0, 0, 0, 0, 0]  # define pose

    while True:
        time.sleep(0.1)
        ans = get_controller_inputs(joy)

        print(ans)
    
    print('End')
