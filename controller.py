import pygame
import time
from math import radians, degrees

def initCont():
    """Inits controller to use it and returns joystick class.
    `returns` `None` if controller is not connected"""
    pygame.init()
    pygame.joystick.init()

    try:  # check if controller is connected
        joystick = pygame.joystick.Joystick(0)  # assign the controller as joystick
        joystick.init()
    except Exception:  # not connected
        return None

    return joystick

def stillConnected():
    """Checks if a Controller is still connected.
    `returns` boolean"""

    pygame.joystick.quit()  # disconnect current controller (if available)
    pygame.joystick.init()  # init controller again

    # check if controller is present
    if pygame.joystick.get_count() == 0:
        return False
    else:
        return True


def listen2Cont(joystick, currPose=[0,0,0,0,0,0]):
    """listens to controller once and returns pose
    `joystick`: Joystick class (given in init function)
    `currPose`: current pose of the robot
    `return`: pose-list with altered values (if return is a string it is a code or an error)
    """
    if joystick is None:  # if no controller was given
        return 'demo'  # return to demo mode

    pos = currPose  # make own varriable to write to

    pos = [pos[0], pos[1], pos[2], degrees(pos[3]), degrees(pos[4]), degrees(pos[5])]  # convert angles to DEG

    # preinitialize increment-values
    dposx, dposy, dposz = 0, 0, 0
    dposaneg, dposapos, dposbneg, dposbpos, dposcneg, dposcpos = 0, 0, 0, 0, 0, 0

    pygame.event.poll()  # get one event
    
    # 0Z----> y  
    # |
    # |
    # \/ x

    dposx = joystick.get_axis(1)        # x-Achse(Linker Joystick unten-> positiv, oben -> negativ)
    dposy = joystick.get_axis(0)        # y-Achse(Linker Joystick rechts -> positiv, links -> negativ)
    dposz = joystick.get_axis(4) *-1    # z-Achse(Rechter Joystick oben -> positiv, unten -> negativ)
    dposaneg = joystick.get_button(15)
    dposapos = joystick.get_button(16)
    dposbneg = joystick.get_button(14)
    dposbpos = joystick.get_button(13)
    dposcneg = joystick.get_button(4)
    dposcpos = joystick.get_button(5)

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

    pos = checkMaxVal(pos, 40, 40, 200, 40, 40, 30)  # this uses degrees

    pos = [pos[0], pos[1], pos[2], radians(pos[3]), radians(pos[4]), radians(pos[5])]  # convert to RAD
    
    return pos


def checkMaxVal(val,maxX,maxY,maxZ,maxA,maxB,maxC):
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
    if val[2] > maxZ:
        val[2] = maxZ
    if val[2] < -maxZ:
        val[2] = -maxZ

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
        ans = listen2Cont(joy, pose)

        if ans is 'stop':
            break
        else:
            print(ans)
    
    print('End')
